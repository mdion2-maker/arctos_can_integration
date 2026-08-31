#!/usr/bin/env python3
"""
joint_streamed_move_test.py

Tests whether a joint needs a continuously-refreshed, incrementally-advancing
target rather than one single far-away command -- which is how the real
JointTrajectoryController -> arctos_hardware_interface path actually drives
motion (arctos_interface.cpp's write() only sends a new CAN frame when the
commanded position changes by more than position_tolerance_, and a real
trajectory publishes a smoothly interpolated, frequently-updated setpoint).

Every single-shot 0xF4/0xF5 test on Joint B during the 2026-08-27 session
produced a partial "burst then stop" result (real motion for a few seconds,
then an effective halt well short of the commanded target, with no
error/protection flag ever set) regardless of speed/accel settings. This
script sends the total move as many small RELATIVE_POSITION increments in
rapid succession instead, to test whether that avoids the same behavior.

Usage:
    python3 joint_streamed_move_test.py --can-id 0x05 --gear-ratio 67.82 \\
        --home-position-deg 12.873 --opposite-limit-deg -24.230 \\
        --total-degrees 2.0 --num-increments 20 --increment-period-s 0.2
"""
import argparse
import sys
import time

import can

READ_ENCODER = 0x31
READ_SHAFT_PROTECTION_STATE = 0x3E
ENABLE_MOTOR = 0xF3
RELATIVE_POSITION = 0xF4
EMERGENCY_STOP = 0xF7

ENCODER_CPR = 16384


def checksum(motor_id, data):
    return (motor_id + sum(data)) & 0xFF


def send_frame(bus, motor_id, data):
    crc = checksum(motor_id, data)
    frame = data + [crc]
    bus.send(can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False))
    return frame


def query(bus, motor_id, opcode, expect_len, timeout=0.4):
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


def move_relative(bus, motor_id, delta_counts, speed, accel):
    pos = int(delta_counts) & 0xFFFFFF
    data = [RELATIVE_POSITION, (speed >> 8) & 0xFF, speed & 0xFF, accel,
             (pos >> 16) & 0xFF, (pos >> 8) & 0xFF, pos & 0xFF]
    return send_frame(bus, motor_id, data)


def emergency_stop(bus, motor_id):
    send_frame(bus, motor_id, [EMERGENCY_STOP])


def main():
    parser = argparse.ArgumentParser(description="Streamed-increment joint move test")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    parser.add_argument("--gear-ratio", type=float, required=True)
    parser.add_argument("--home-position-deg", type=float, required=True)
    parser.add_argument("--opposite-limit-deg", type=float, required=True)
    parser.add_argument("--safety-margin-deg", type=float, default=2.0)
    parser.add_argument("--total-degrees", type=float, default=2.0,
                         help="Signed total move size, in raw-count-equivalent joint degrees")
    parser.add_argument("--num-increments", type=int, default=20)
    parser.add_argument("--increment-period-s", type=float, default=0.2)
    parser.add_argument("--speed", type=int, default=15)
    parser.add_argument("--accel", type=int, default=1)
    args = parser.parse_args()

    lo = args.opposite_limit_deg + args.safety_margin_deg
    hi = args.home_position_deg - args.safety_margin_deg

    def within_safe_band(deg):
        return lo <= deg <= hi

    total_counts = round(args.total_degrees * args.gear_ratio * ENCODER_CPR / 360.0)
    increment_counts = round(total_counts / args.num_increments)

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    try:
        raw0 = read_encoder(bus, args.can_id)
        deg0 = joint_deg(raw0, args.gear_ratio)
        print(f"Start: raw48={raw0}  joint_deg={deg0:.4f}")
        print(f"Safe band: [{lo:.3f}, {hi:.3f}] deg")

        worst_case_deg = deg0 + args.total_degrees
        if not within_safe_band(worst_case_deg):
            print(f"ABORT: worst-case final position {worst_case_deg:.3f} deg would leave the safe band.")
            return

        print(f"Sending {args.num_increments} increments of {increment_counts} raw counts "
              f"every {args.increment_period_s}s (total {total_counts} counts, "
              f"~{args.total_degrees} deg)\n")

        print("Sending ENABLE_MOTOR...")
        send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x01])
        time.sleep(0.3)

        for i in range(1, args.num_increments + 1):
            move_relative(bus, args.can_id, increment_counts, args.speed, args.accel)
            time.sleep(args.increment_period_s)
            raw_now = read_encoder(bus, args.can_id)
            if raw_now is None:
                print(f"  [{i}/{args.num_increments}] WARNING: encoder read timeout")
                continue
            deg_now = joint_deg(raw_now, args.gear_ratio)
            prot = query(bus, args.can_id, READ_SHAFT_PROTECTION_STATE, 3)
            print(f"  [{i}/{args.num_increments}] raw48={raw_now:>10}  joint_deg={deg_now:.4f}  "
                  f"protection={list(prot) if prot else None}")
            if prot is not None and len(prot) >= 2 and prot[1] not in (0,):
                print("  Shaft protection flag set -- emergency stopping.")
                emergency_stop(bus, args.can_id)
                sys.exit(1)
            if not within_safe_band(deg_now):
                print("  Leaving safe band -- emergency stopping.")
                emergency_stop(bus, args.can_id)
                sys.exit(1)

        raw1 = read_encoder(bus, args.can_id)
        deg1 = joint_deg(raw1, args.gear_ratio)
        moved = deg1 - deg0
        pct_complete = 100.0 * (raw1 - raw0) / total_counts if total_counts else 0.0
        print(f"\nFinal: raw48={raw1}  joint_deg={deg1:.4f}")
        print(f"Moved {moved:+.4f} deg total ({pct_complete:.1f}% of the commanded {args.total_degrees} deg)")

    finally:
        print("\nDisabling motor coils.")
        send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x00])
        bus.shutdown()


if __name__ == "__main__":
    main()
