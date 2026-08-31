#!/usr/bin/env python3
"""
joint_stepped_move_test.py

Monitored, multi-step move test for a single Arctos joint. Generic across
joints -- pass --can-id, --gear-ratio, and the joint's calibrated
home_position/opposite_limit (in joint degrees, from arctos_controller.yaml)
so the script can refuse to send any step that would leave the confirmed-safe
envelope.

Developed while investigating Joint B's "0xF5/0xF4 commands get acknowledged
(status=1, 'movement started') but only produce a partial burst of real
motion before stopping" behavior (2026-08-27 session). Supports both
ABSOLUTE_POSITION (0xF5) and RELATIVE_POSITION (0xF4) -- the latter has zero
confirmed usage in the real motor_driver.cpp source, so treat it as more
speculative than 0xF5.

UPDATE 2026-08-27 (later same day): ENABLE_SHAFT_PROTECTION (0x88) -- never
sent anywhere in this project before this session -- was suspected as the
cause of "started but didn't finish" moves, since it is required for
READ_SHAFT_PROTECTION_STATE (0x3E) to detect and report a real stall.
Enabling it did trip 0x3E immediately. But a direct comparison -- the
identical command, run again with 0x88 NOT sent -- moved almost perfectly.
That refutes "genuine stall" as a confirmed conclusion; 0x88 appears capable
of a false-positive trip, not just a real one. It is OFF BY DEFAULT here
(--enable-shaft-protection to turn it on) until this is better understood.
A trip also latches the driver down (see this document's Section 1) and
requires a power-cycle to clear, expensive to trigger by accident.

CAUTION (2026-08-31) -- --mode absolute is suspected unsafe once a joint's
raw counter has drifted far from zero. joint_micro_move_test.py used this
exact pattern (pos_field = an absolute raw target, packed via a plain
`& 0xFFFFFF`) and caused a real incident on Joint B: a small intended nudge
turned into an enormous, non-settling, uninterrupted move once the joint's
absolute position had drifted to roughly -274,000 counts from earlier
un-homed testing, requiring motor power to be cut. This script's absolute
mode sends `pos_field = target_raw` (= current_raw + step_counts) through
the identical unguarded packing -- it has the same exposure and has not yet
been fixed or re-verified safe. --mode relative sends only a small signed
`step_counts` delta, which is lower-risk but still packs a possibly-negative
value the same way, rather than splitting it into a direction byte plus an
always-positive magnitude like the proven-safe pattern in
joint_statistical_reliability_test.py / joint_position_control_test.py
(POSITION_CONTROL, 0xFD). Until this script is updated to match that
pattern, prefer those two for actual motion testing, and treat any past
--mode absolute result run against a joint with an unknown/large drifted
position as unverified.

Usage:
    python3 joint_stepped_move_test.py --can-id 0x05 --gear-ratio 67.82 \\
        --home-position-deg 12.873 --opposite-limit-deg -24.230 \\
        --mode relative --step-degrees 2.0 --num-steps 3 --speed 15 --accel 1
"""
import argparse
import sys
import time

import can

READ_ENCODER = 0x31
READ_SHAFT_PROTECTION_STATE = 0x3E
ENABLE_MOTOR = 0xF3
ENABLE_SHAFT_PROTECTION = 0x88
RELATIVE_POSITION = 0xF4
ABSOLUTE_POSITION = 0xF5
EMERGENCY_STOP = 0xF7

ENCODER_CPR = 16384


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
        if resp is None:
            continue
        if resp.arbitration_id == motor_id and len(resp.data) >= expect_len and resp.data[0] == opcode:
            return resp.data
    return None


def read_encoder(bus, motor_id):
    d = query(bus, motor_id, READ_ENCODER, 8)
    if d is None:
        return None
    return int.from_bytes(d[1:7], byteorder="big", signed=True)


def joint_deg(raw48, gear_ratio):
    return (raw48 * 360.0 / ENCODER_CPR) / gear_ratio


def send_move(bus, motor_id, opcode, pos_field, speed, accel):
    pos = int(pos_field) & 0xFFFFFF
    data = [opcode, (speed >> 8) & 0xFF, speed & 0xFF, accel,
             (pos >> 16) & 0xFF, (pos >> 8) & 0xFF, pos & 0xFF]
    return send_frame(bus, motor_id, data)


def emergency_stop(bus, motor_id):
    send_frame(bus, motor_id, [EMERGENCY_STOP])


def main():
    parser = argparse.ArgumentParser(description="Monitored multi-step joint move test")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    parser.add_argument("--gear-ratio", type=float, required=True)
    parser.add_argument("--home-position-deg", type=float, required=True,
                         help="Calibrated home_position from arctos_controller.yaml, in degrees")
    parser.add_argument("--opposite-limit-deg", type=float, required=True,
                         help="Calibrated opposite_limit from arctos_controller.yaml, in degrees")
    parser.add_argument("--safety-margin-deg", type=float, default=2.0)
    parser.add_argument("--mode", choices=["absolute", "relative"], default="relative")
    parser.add_argument("--step-degrees", type=float, default=2.0,
                         help="Signed step size per command, in raw-count-equivalent joint degrees")
    parser.add_argument("--num-steps", type=int, default=3)
    parser.add_argument("--speed", type=int, default=15)
    parser.add_argument("--accel", type=int, default=1)
    parser.add_argument("--inter-step-pause-s", type=float, default=2.0)
    parser.add_argument("--re-enable-each-step", action="store_true",
                         help="Re-send ENABLE_MOTOR before every step, not just once at the start")
    parser.add_argument("--enable-shaft-protection", action="store_true",
                         help="Send ENABLE_SHAFT_PROTECTION (0x88) at startup. CAUTION: on "
                              "2026-08-27 this caused instant trips on commands that moved "
                              "perfectly when protection was NOT enabled -- treat trips as a "
                              "possible false positive, not confirmed proof of a real stall, "
                              "until cross-checked against an unprotected run of the same command. "
                              "Off by default for that reason.")
    args = parser.parse_args()

    lo = min(args.home_position_deg, args.opposite_limit_deg) + args.safety_margin_deg
    hi = max(args.home_position_deg, args.opposite_limit_deg) - args.safety_margin_deg

    def within_safe_band(deg):
        return lo <= deg <= hi

    step_counts = round(args.step_degrees * args.gear_ratio * ENCODER_CPR / 360.0)

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    try:
        raw0 = read_encoder(bus, args.can_id)
        deg0 = joint_deg(raw0, args.gear_ratio)
        print(f"Start: raw48={raw0}  joint_deg={deg0:.4f}")

        if args.enable_shaft_protection:
            print("Sending ENABLE_SHAFT_PROTECTION (0x88, 0x01) -- CAUTION: this has caused "
                  "instant false-positive trips on commands that otherwise moved correctly. "
                  "See the SOP's session log before trusting a trip as a real stall.")
            send_frame(bus, args.can_id, [ENABLE_SHAFT_PROTECTION, 0x01])
            time.sleep(0.3)

        print(f"Safe band: [{lo:.3f}, {hi:.3f}] deg")
        print(f"Mode={args.mode}  step={args.step_degrees} deg ({step_counts} raw counts)  "
              f"num_steps={args.num_steps}  speed={args.speed}  accel={args.accel}\n")

        print("Sending ENABLE_MOTOR...")
        send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x01])
        time.sleep(0.3)

        current_raw = raw0
        for i in range(1, args.num_steps + 1):
            prev_deg = joint_deg(current_raw, args.gear_ratio)

            if args.mode == "absolute":
                target_raw = current_raw + step_counts
                projected_deg = joint_deg(target_raw, args.gear_ratio)
                pos_field = target_raw
                opcode = ABSOLUTE_POSITION
            else:
                projected_deg = prev_deg + args.step_degrees
                pos_field = step_counts
                opcode = RELATIVE_POSITION

            if not within_safe_band(projected_deg):
                print(f"[Step {i}] ABORT: projected {projected_deg:.3f} deg would leave the safe band.")
                break

            if args.re_enable_each_step:
                send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x01])
                time.sleep(0.3)

            frame = send_move(bus, args.can_id, opcode, pos_field, args.speed, args.accel)
            print(f"[Step {i}] Sent {'ABSOLUTE' if opcode == ABSOLUTE_POSITION else 'RELATIVE'} "
                  f"pos_field={pos_field}: {[hex(b) for b in frame]}")

            for _ in range(10):
                time.sleep(0.5)
                raw_now = read_encoder(bus, args.can_id)
                if raw_now is None:
                    print("    WARNING: encoder read timeout")
                    continue
                deg_now = joint_deg(raw_now, args.gear_ratio)
                prot = query(bus, args.can_id, READ_SHAFT_PROTECTION_STATE, 3)
                print(f"    raw48={raw_now:>10}  joint_deg={deg_now:.4f}  "
                      f"protection={list(prot) if prot else None}")
                if prot is not None and len(prot) >= 2 and prot[1] not in (0,):
                    print(f"    *** Shaft protection TRIPPED (status={prot[1]}) -- a genuine stall. ***")
                    print("    The driver is now latched down. Power-cycle its main supply to clear it.")
                    emergency_stop(bus, args.can_id)
                    sys.exit(1)
                if not within_safe_band(deg_now):
                    print("    Left the safe band -- emergency stopping.")
                    emergency_stop(bus, args.can_id)
                    sys.exit(1)

            current_raw = read_encoder(bus, args.can_id) or current_raw
            current_deg = joint_deg(current_raw, args.gear_ratio)
            moved = current_deg - prev_deg
            print(f"    Step {i} result: now at {current_deg:.4f} deg "
                  f"(moved {moved:+.4f}, commanded {args.step_degrees:+.4f})\n")
            time.sleep(args.inter_step_pause_s)

        final_raw = read_encoder(bus, args.can_id)
        final_deg = joint_deg(final_raw, args.gear_ratio)
        print(f"=== Final: raw48={final_raw}  joint_deg={final_deg:.4f}  "
              f"(started {deg0:.4f}, net change {final_deg - deg0:+.4f}) ===")

    finally:
        print("\nDisabling motor coils.")
        send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x00])
        bus.shutdown()


if __name__ == "__main__":
    main()
