#!/usr/bin/env python3
"""
joint_long_move_monitor.py

Sends a single, potentially large POSITION_CONTROL (0xFD) command and polls
the encoder at fine time resolution (default every 2s) for up to
--max-wait-s, printing a live progress trace with instantaneous rate
(counts/sec) between polls. Built after single large commands turned out to
reveal information that short 5-10s test windows miss entirely.

On 2026-08-27, this tool's live trace is what revealed that a "stuck"
15-degree move on Joint B was not actually stuck -- it was executing a
stick-slip pattern (long stalls of 10-30+ seconds, punctuated by sudden
multi-degree bursts) that a short observation window would have reported as
a simple failure. Use this whenever a move looks stalled: watch the rate
column for several poll cycles before assuming nothing is happening.

Deliberately does NOT gate against a calibrated home_position/opposite_limit
envelope -- pass --home-position-deg/--opposite-limit-deg matching the
current physical configuration's real limits (or a very wide placeholder
pair like 1000/-1000 if the bench setup is confirmed to allow free rotation,
as Joint B's was on 2026-08-27) since this tool is specifically meant for
distances well beyond what small-move scripts in this package are gated for.

Usage:
    python3 joint_long_move_monitor.py --can-id 0x05 --gear-ratio 67.82 \\
        --home-position-deg 12.873 --opposite-limit-deg -24.230 \\
        --step-degrees 15.0 --speed 15 --accel 1
"""
import argparse
import sys
import time

import can

READ_ENCODER = 0x31
ENABLE_MOTOR = 0xF3
POSITION_CONTROL = 0xFD
EMERGENCY_STOP = 0xF7

ENCODER_CPR = 16384
MICROSTEPS_PER_MOTOR_REV = 3200.0


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


def emergency_stop(bus, motor_id):
    send_frame(bus, motor_id, [EMERGENCY_STOP])


def main():
    parser = argparse.ArgumentParser(description="Monitor a single long move with a live progress trace")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    parser.add_argument("--gear-ratio", type=float, required=True)
    parser.add_argument("--home-position-deg", type=float, required=True)
    parser.add_argument("--opposite-limit-deg", type=float, required=True)
    parser.add_argument("--safety-margin-deg", type=float, default=2.0)
    parser.add_argument("--step-degrees", type=float, default=15.0,
                         help="Signed distance to command; sign sets direction")
    parser.add_argument("--speed", type=int, default=15)
    parser.add_argument("--accel", type=int, default=1)
    parser.add_argument("--poll-period-s", type=float, default=2.0)
    parser.add_argument("--max-wait-s", type=float, default=180.0)
    args = parser.parse_args()

    lo = args.opposite_limit_deg + args.safety_margin_deg
    hi = args.home_position_deg - args.safety_margin_deg

    def within_safe_band(deg):
        return lo <= deg <= hi

    direction = 0x80 if args.step_degrees < 0 else 0x00
    pulses = round(abs(args.step_degrees) * MICROSTEPS_PER_MOTOR_REV * args.gear_ratio / 360.0)

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    try:
        raw0 = read_encoder(bus, args.can_id)
        deg0 = joint_deg(raw0, args.gear_ratio)
        print(f"Start: raw48={raw0}  joint_deg={deg0:.4f}", flush=True)
        print(f"Safe band: [{lo:.3f}, {hi:.3f}] deg", flush=True)

        projected = deg0 + args.step_degrees
        if not within_safe_band(projected):
            print(f"ABORT: projected {projected:.3f} deg would leave the safe band.", flush=True)
            return

        print(f"Commanding {args.step_degrees:+.1f} deg ({pulses} pulses), "
              f"speed={args.speed} accel={args.accel}\n", flush=True)

        send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x01])
        time.sleep(0.3)
        while bus.recv(timeout=0.0) is not None:
            pass

        t_start = time.time()
        frame = move_position_control(bus, args.can_id, pulses, args.speed, args.accel, direction)
        print(f"Sent: {[hex(b) for b in frame]}", flush=True)

        statuses_seen = []
        last_raw = raw0
        try:
            while time.time() - t_start < args.max_wait_s:
                time.sleep(args.poll_period_s)
                statuses_seen.extend(watch_fd_status_frames(bus, args.can_id, 0.0))
                raw_now = read_encoder(bus, args.can_id)
                if raw_now is None:
                    print("  WARNING: encoder read timeout", flush=True)
                    continue
                deg_now = joint_deg(raw_now, args.gear_ratio)
                elapsed = time.time() - t_start
                rate = (raw_now - last_raw) / args.poll_period_s
                traveled = deg_now - deg0
                pct = 100.0 * traveled / args.step_degrees if args.step_degrees else 0.0
                print(f"  t+{elapsed:6.1f}s  raw48={raw_now:>10}  traveled={traveled:8.3f} deg "
                      f"({pct:5.1f}%)  rate={rate:+.1f} counts/s  statuses_so_far={statuses_seen}",
                      flush=True)
                last_raw = raw_now
                if not within_safe_band(deg_now):
                    print("  Left the safe band -- emergency stopping.", flush=True)
                    emergency_stop(bus, args.can_id)
                    sys.exit(1)
                if abs(pct) >= 99.0:
                    print("\nReached ~100% of commanded distance.", flush=True)
                    break
        except KeyboardInterrupt:
            print("\nCtrl+C -- emergency stop", flush=True)
            emergency_stop(bus, args.can_id)
            sys.exit(1)

        final_raw = read_encoder(bus, args.can_id)
        final_deg = joint_deg(final_raw, args.gear_ratio)
        elapsed_total = time.time() - t_start
        print(f"\n=== Final: traveled {final_deg - deg0:.3f} deg of {args.step_degrees} commanded, "
              f"after {elapsed_total:.1f}s. FD statuses seen: {statuses_seen} ===", flush=True)

    finally:
        print("\nDisabling motor coils.", flush=True)
        send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x00])
        bus.shutdown()


if __name__ == "__main__":
    main()
