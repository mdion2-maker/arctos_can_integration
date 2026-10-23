#!/usr/bin/env python3
"""
joint_calibrate.py

Measure the travel between a joint's two endstops.

    python3 joint_calibrate.py z
    python3 joint_calibrate.py y --speed 25

Sweeps clockwise until a limit trips, backs off, sweeps counter-clockwise until
the other trips, and reports the separation. Joint parameters -- CAN ID, gear
ratio, which byte is clockwise, which IO bits are real -- come from
joint_move.py so there is one table, not two that drift apart.

BOTH approaches are made at the SAME speed, and that is the point of the script.
Trigger position varies with approach speed, and the two effects compound: a
faster approach records the clockwise limit further clockwise AND the
counter-clockwise limit further counter-clockwise, inflating the separation from
both ends. Joint Y measured 178.6 deg at speed 150, 165.5 at ~50, and 158.2 at
25, converging as the speed dropped.

Run it at a speed the drivetrain can actually transmit. Slipping gears turn more
motor than joint, so the encoder over-reports the distance -- Y's envelope came
out 20 deg too large at speed 150 for exactly that reason. If two runs at the
same speed agree closely, the number is trustworthy; Y's two creep runs agreed to
0.030 deg, which is what established 158.24 as real rather than another estimate.
"""
import argparse
import collections
import sys
import time

import can

from joint_move import (JOINTS, io_mask, checksum, send, query,
                        ENCODER_CPR, MICROSTEPS_PER_REV, ACCEL,
                        STALL_WINDOW_S, STALL_GRACE_S, STALL_FRACTION)


def sweep(bus, can_id, gear, mask, deg, direction, speed, label, watch=True):
    """Travel up to deg in one direction. Returns (raw, tripped_bit or None)."""
    p0 = query(bus, can_id, 0x31)
    io0 = query(bus, can_id, 0x34)
    if p0 is None or io0 is None:
        sys.exit(f"No reply from 0x{can_id:02X}.")
    armed = io0 & mask
    cpjd = gear * ENCODER_CPR / 360.0
    pulses = round(deg * gear / 360.0 * MICROSTEPS_PER_REV)
    expected_s = deg * gear / 360.0 / speed * 60.0
    stall_min = max(200, int(speed / 60.0 * ENCODER_CPR * STALL_WINDOW_S * STALL_FRACTION))

    print(f"\n>>> {label}  (up to {deg:g} deg, ~{expected_s:.0f}s)", flush=True)
    send(bus, can_id, [0xFD, direction + ((speed >> 8) & 0x0F), speed & 0xFF, ACCEL,
                       (pulses >> 16) & 0xFF, (pulses >> 8) & 0xFF, pulses & 0xFF])
    hist = collections.deque()
    t0 = last_print = time.time()
    while time.time() - t0 < expected_s + 30:
        io = query(bus, can_id, 0x34)
        pos = query(bus, can_id, 0x31)
        now = time.time()
        if io is None or pos is None:
            continue
        armed |= (io & mask)
        newly_low = armed & ~io & mask
        if watch and newly_low:
            send(bus, can_id, [0xF7])
            bit = 0 if newly_low & 1 else 1
            print(f"    IN_{bit + 1} tripped at {now - t0:.1f}s  raw={pos}  "
                  f"{(pos - p0) / cpjd:+.3f} deg", flush=True)
            return pos, bit
        hist.append((now, pos))
        while hist and now - hist[0][0] > STALL_WINDOW_S:
            hist.popleft()
        if (now - t0 > STALL_GRACE_S and len(hist) > 5
                and abs(pos - hist[0][1]) < stall_min):
            send(bus, can_id, [0xF7])
            print(f"    STALL at raw={pos}  {(pos - p0) / cpjd:+.3f} deg", flush=True)
            return pos, None
        if now - last_print > 20.0:
            print(f"    t={now - t0:5.0f}s  {(pos - p0) / cpjd:+8.2f} deg  io=0x{io:02X}",
                  flush=True)
            last_print = now
        if abs(pos - p0) >= pulses * ENCODER_CPR / MICROSTEPS_PER_REV - 300:
            print(f"    bound reached at {(pos - p0) / cpjd:+.3f} deg", flush=True)
            return pos, None
    return query(bus, can_id, 0x31), None


def main():
    ap = argparse.ArgumentParser(description="Measure a joint's endstop-to-endstop travel")
    ap.add_argument("joint", choices=sorted(JOINTS))
    ap.add_argument("--speed", type=int, default=25, help="motor rpm for BOTH approaches")
    ap.add_argument("--backoff", type=float, default=10.0, help="degrees to clear the first limit")
    ap.add_argument("--max", type=float, default=200.0, help="search bound per direction")
    ap.add_argument("--channel", default="can0")
    args = ap.parse_args()

    can_id, gear, board, cw_byte, remapped, max_speed = JOINTS[args.joint]
    if max_speed is not None and args.speed > max_speed:
        print(f"*** speed {args.speed} exceeds joint {args.joint.upper()}'s measured safe "
              f"maximum of {max_speed}; clamping. Above it the gears slip, which inflates "
              f"the measured envelope.")
        args.speed = max_speed
    if cw_byte is None:
        sys.exit(f"Joint {args.joint.upper()} has no confirmed direction mapping.")
    ccw_byte = cw_byte ^ 0x80
    mask = io_mask(board, remapped)
    cpjd = gear * ENCODER_CPR / 360.0

    print(f"calibrating joint {args.joint.upper()}  CAN 0x{can_id:02X}  {board}  {gear}:1"
          f"{'  (remap ON)' if remapped else ''}")
    print(f"both approaches at speed {args.speed}, watching IO bits "
          f"{[b for b in (0, 1) if (mask >> b) & 1]}")

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    try:
        send(bus, can_id, [0xF3, 0x01])
        time.sleep(0.3)

        first, bit_a = sweep(bus, can_id, gear, mask, args.max, cw_byte, args.speed,
                             "sweep CLOCKWISE to the first limit")
        if bit_a is None:
            sys.exit("No limit found clockwise -- nothing to measure.")
        sweep(bus, can_id, gear, mask, args.backoff, ccw_byte, args.speed,
              f"back off {args.backoff:g} deg", watch=False)
        time.sleep(0.5)
        second, bit_b = sweep(bus, can_id, gear, mask, args.max, ccw_byte, args.speed,
                              "sweep COUNTER-CLOCKWISE to the other limit")
        if bit_b is None:
            sys.exit("No limit found counter-clockwise -- nothing to measure.")
        if bit_a == bit_b:
            print(f"\nWARNING: both sweeps tripped IN_{bit_a + 1}. That is one sensor seen "
                  f"twice, not two limits -- the separation below is meaningless.")

        sep = abs(second - first)
        print("\n=== result ===")
        print(f"  clockwise limit          IN_{bit_a + 1}   raw = {first}")
        print(f"  counter-clockwise limit  IN_{bit_b + 1}   raw = {second}")
        print(f"  separation = {sep} counts = {sep / cpjd:.3f} deg")
        print(f"\n  Repeat at the same speed. Two runs agreeing closely means the number is")
        print(f"  real; disagreement means the gears are slipping at speed {args.speed}.")
    finally:
        send(bus, can_id, [0xF7])
        time.sleep(0.1)
        send(bus, can_id, [0xF3, 0x00])
        end_pos = query(bus, can_id, 0x31)
        end_io = query(bus, can_id, 0x34)
        pos_s = str(end_pos) if end_pos is not None else "no reply"
        io_s = f"0x{end_io:02X}" if end_io is not None else "no reply"
        print(f"\nfinal raw={pos_s}  io={io_s}.  coils disabled")
        bus.shutdown()


if __name__ == "__main__":
    main()
