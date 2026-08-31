#!/usr/bin/env python3
"""
joint_statistical_reliability_test.py

Statistical reliability test for a single Arctos joint: N repetitions of the
same small POSITION_CONTROL (0xFD) command, either alternating direction
(stays near its starting position) or sustained in one direction (travels
progressively further from start). Logs FD status frames and actual
movement for every repetition to establish a real success rate, rather than
judging reliability from a single try.

Developed 2026-08-27 on Joint B after single-command tests kept looking
inconsistent. The key finding this tool is built to surface: an alternating
test can show 100% reliability while a sustained one-direction test on the
exact same joint, current, and speed degrades sharply after a few reps --
which is the signature of a real, position-localized issue (a mechanical
catch, in Joint B's case) rather than a current/speed/protocol problem.
If a sustained run fails but an alternating run does not, narrow the
sustained run's --step-degrees and starting position to localize where the
failures start, then check that specific range by hand, slowly.

ENABLE_SHAFT_PROTECTION is deliberately NOT sent by this script -- on
2026-08-27 it produced a false-positive trip on a command that moved
correctly when it was left off. Use joint_position_control_test.py's
--enable-shaft-protection flag separately if you want to investigate that
feature on its own.

Usage:
    # Alternating (tests whether the link/settings are reliable in general)
    python3 joint_statistical_reliability_test.py --can-id 0x05 --gear-ratio 67.82 \\
        --home-position-deg 12.873 --opposite-limit-deg -24.230 \\
        --step-degrees 0.5 --num-reps 20 --mode alternating --speed 15 --accel 1

    # Sustained (tests for a position-localized issue across a range)
    python3 joint_statistical_reliability_test.py --can-id 0x05 --gear-ratio 67.82 \\
        --home-position-deg 12.873 --opposite-limit-deg -24.230 \\
        --step-degrees -0.5 --num-reps 10 --mode sustained --speed 15 --accel 1
"""
import argparse
import time

import can

READ_ENCODER = 0x31
ENABLE_MOTOR = 0xF3
POSITION_CONTROL = 0xFD

ENCODER_CPR = 16384
MICROSTEPS_PER_MOTOR_REV = 3200.0  # 200 full steps/rev x 16 microsteps/step


def checksum(motor_id, data):
    return (motor_id + sum(data)) & 0xFF


def send_frame(bus, motor_id, data):
    crc = checksum(motor_id, data)
    frame = data + [crc]
    bus.send(can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False))
    return frame


def query(bus, motor_id, opcode, expect_len, timeout=0.5):
    while bus.recv(timeout=0.0) is not None:
        pass
    send_frame(bus, motor_id, [opcode])
    end = time.time() + timeout
    while time.time() < end:
        resp = bus.recv(timeout=max(0.0, end - time.time()))
        if resp and resp.arbitration_id == motor_id and len(resp.data) >= expect_len and resp.data[0] == opcode:
            return list(resp.data)
    return None


def read_encoder(bus, motor_id):
    d = query(bus, motor_id, READ_ENCODER, 8)
    if d is None:
        return None
    return int.from_bytes(bytes(d[1:7]), byteorder="big", signed=True)


def joint_deg(raw48, gear_ratio):
    return (raw48 * 360.0 / ENCODER_CPR) / gear_ratio


def move_position_control(bus, motor_id, pulses, speed, accel, direction):
    pulses = int(pulses) & 0xFFFFFF
    data = [POSITION_CONTROL, direction + ((speed >> 8) & 0b1111), speed & 0xFF, accel,
             (pulses >> 16) & 0xFF, (pulses >> 8) & 0xFF, pulses & 0xFF]
    return send_frame(bus, motor_id, data)


def watch_fd_status_frames(bus, motor_id, timeout):
    seen = []
    end = time.time() + timeout
    while time.time() < end:
        resp = bus.recv(timeout=max(0.0, end - time.time()))
        if resp is None:
            continue
        if resp.arbitration_id == motor_id and len(resp.data) == 3 and resp.data[0] == POSITION_CONTROL:
            seen.append(resp.data[1])
    return seen


def main():
    parser = argparse.ArgumentParser(description="Statistical joint move reliability test")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    parser.add_argument("--gear-ratio", type=float, required=True)
    parser.add_argument("--home-position-deg", type=float, required=True)
    parser.add_argument("--opposite-limit-deg", type=float, required=True)
    parser.add_argument("--safety-margin-deg", type=float, default=3.0)
    parser.add_argument("--step-degrees", type=float, default=0.5,
                         help="Signed magnitude per rep; sign sets direction for 'sustained' mode")
    parser.add_argument("--num-reps", type=int, default=10)
    parser.add_argument("--mode", choices=["alternating", "sustained"], default="alternating",
                         help="alternating: direction flips each rep, stays near start. "
                              "sustained: same direction every rep, travels progressively further.")
    parser.add_argument("--speed", type=int, default=15)
    parser.add_argument("--accel", type=int, default=1)
    parser.add_argument("--inter-rep-pause-s", type=float, default=2.0)
    parser.add_argument("--status-timeout-s", type=float, default=4.0)
    args = parser.parse_args()

    lo = args.opposite_limit_deg + args.safety_margin_deg
    hi = args.home_position_deg - args.safety_margin_deg

    def within_safe_band(deg):
        return lo <= deg <= hi

    step_pulses = round(abs(args.step_degrees) * MICROSTEPS_PER_MOTOR_REV * args.gear_ratio / 360.0)

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    results = []
    try:
        raw0 = read_encoder(bus, args.can_id)
        deg0 = joint_deg(raw0, args.gear_ratio)
        print(f"Start: raw48={raw0}  joint_deg={deg0:.4f}")
        print(f"Safe band: [{lo:.3f}, {hi:.3f}] deg")
        print(f"Mode={args.mode}  {args.num_reps} reps of {abs(args.step_degrees)} deg, "
              f"speed={args.speed} accel={args.accel}, shaft protection OFF\n")

        prev_deg = deg0
        for i in range(1, args.num_reps + 1):
            if args.mode == "alternating":
                direction_sign = 1 if i % 2 == 1 else -1
            else:
                direction_sign = 1 if args.step_degrees >= 0 else -1
            direction_byte = 0x00 if direction_sign > 0 else 0x80
            expected = direction_sign * abs(args.step_degrees)

            projected = prev_deg + expected
            if not within_safe_band(projected):
                print(f"[Rep {i}] ABORT: projected {projected:.3f} deg would leave the safe band.")
                break

            send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x01])
            time.sleep(0.3)
            while bus.recv(timeout=0.0) is not None:
                pass

            move_position_control(bus, args.can_id, step_pulses, args.speed, args.accel, direction_byte)
            statuses = watch_fd_status_frames(bus, args.can_id, args.status_timeout_s)

            raw_now = read_encoder(bus, args.can_id)
            deg_now = joint_deg(raw_now, args.gear_ratio) if raw_now is not None else None
            moved = (deg_now - prev_deg) if deg_now is not None else None

            success = 2 in statuses
            pct = (100.0 * moved / expected) if (moved is not None and expected != 0) else None
            pct_str = f"{pct:.0f}%" if pct is not None else "N/A"
            print(f"[Rep {i:2d}] dir={'+' if direction_sign > 0 else '-'}  statuses={statuses}  "
                  f"moved={moved:+.4f} deg  expected={expected:+.4f}  "
                  f"pct={pct_str}  {'OK' if success else 'NO-FD02'}")

            results.append({"rep": i, "statuses": statuses, "moved": moved,
                             "expected": expected, "success": success})

            if deg_now is not None:
                prev_deg = deg_now
            time.sleep(args.inter_rep_pause_s)

        successes = sum(1 for r in results if r["success"])
        total = len(results)
        if total:
            print(f"\n=== Summary: {successes}/{total} reps got FD 02 ({100.0*successes/total:.1f}%) ===")
            half = total // 2
            first_half, second_half = results[:half], results[half:]
            if first_half and second_half:
                fh_rate = 100.0 * sum(1 for r in first_half if r["success"]) / len(first_half)
                sh_rate = 100.0 * sum(1 for r in second_half if r["success"]) / len(second_half)
                print(f"First half success rate: {fh_rate:.1f}%  |  Second half success rate: {sh_rate:.1f}%")

        final_raw = read_encoder(bus, args.can_id)
        final_deg = joint_deg(final_raw, args.gear_ratio)
        print(f"Final position: {final_deg:.4f} deg (started {deg0:.4f}, net change {final_deg-deg0:+.4f})")

    finally:
        print("\nDisabling motor coils.")
        send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x00])
        bus.shutdown()


if __name__ == "__main__":
    main()
