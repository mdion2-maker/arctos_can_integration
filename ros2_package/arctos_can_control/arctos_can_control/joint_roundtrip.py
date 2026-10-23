#!/usr/bin/env python3
"""
joint_roundtrip.py

Move each joint out and back, and report whether it returned to where it started.

    python3 joint_roundtrip.py 45
    python3 joint_roundtrip.py 10 --joints x,z
    python3 joint_roundtrip.py 45 --dry-run

A health check across the arm: each joint travels the given distance clockwise,
then the same distance counter-clockwise, and the summary reports how far from
its starting position it ended up. Joints that are not on the bus are skipped
rather than treated as failures, so this works with a partly assembled arm.

Read the return error, not the delivered distance. Returning to the start is
the property worth testing -- it exercises both directions, and a joint that
consistently comes home has no gross backlash or lost steps in that range.

Two things it cannot tell you. It cannot confirm the ARM moved: the encoder is
on the motor shaft, upstream of the reduction, so slipping gears give a flawless
trace for an output going nowhere. And a leg that stops early on an endstop
makes the return error meaningless -- the summary flags those rather than
reporting a number that looks like a measurement.

Per-joint speed limits come from the JOINTS table and are enforced, so a joint
whose gears slip above a known speed cannot be driven past it from here.
"""
import argparse
import collections
import sys
import time

import can

from joint_move import (JOINTS, io_mask, send, query,
                        ENCODER_CPR, MICROSTEPS_PER_REV, ACCEL,
                        STALL_WINDOW_S, STALL_GRACE_S, STALL_FRACTION)

SETTLE_COUNTS = 5
SETTLE_SAMPLES = 3


def leg(bus, can_id, gear, mask, deg, direction, speed, label):
    """One leg. Returns (end_raw, note) where note is None on a clean arrival."""
    p0 = query(bus, can_id, 0x31)
    io0 = query(bus, can_id, 0x34)
    if p0 is None or io0 is None:
        return None, "no reply"
    armed = io0 & mask
    cpjd = gear * ENCODER_CPR / 360.0
    pulses = round(deg * gear / 360.0 * MICROSTEPS_PER_REV)
    expected_s = deg * gear / 360.0 / speed * 60.0

    send(bus, can_id, [0xFD, direction + ((speed >> 8) & 0x0F), speed & 0xFF, ACCEL,
                       (pulses >> 16) & 0xFF, (pulses >> 8) & 0xFF, pulses & 0xFF])
    t0 = time.time()
    hist = collections.deque()
    stall_min = max(200, int(speed / 60.0 * ENCODER_CPR * STALL_WINDOW_S * STALL_FRACTION))
    while time.time() - t0 < expected_s + 25:
        io = query(bus, can_id, 0x34)
        pos = query(bus, can_id, 0x31)
        now = time.time()
        if io is None or pos is None:
            continue
        armed |= (io & mask)
        newly_low = armed & ~io & mask
        if newly_low:
            send(bus, can_id, [0xF7])
            bit = 0 if newly_low & 1 else 1
            end = query(bus, can_id, 0x31)
            print(f"    {label}: ENDSTOP IN_{bit+1} after {(end-p0)/cpjd:+.3f} deg", flush=True)
            return end, f"stopped on IN_{bit+1}"
        # Stall detection must be windowed and must not start during the
        # acceleration ramp. An earlier version here compared consecutive polls
        # (under 5 counts, three times running) with no grace period, and at
        # ACCEL=2 the ramp is slow enough that the first few polls of every move
        # look motionless -- it reported Joint A as stalled after 0.246 deg,
        # minutes after that same joint completed a clean 360 deg round trip.
        # This is joint_move.py's logic: measure travel across a 1.5s window,
        # and only after the ramp has had 4s to get going.
        hist.append((now, pos))
        while hist and now - hist[0][0] > STALL_WINDOW_S:
            hist.popleft()
        if (now - t0 > STALL_GRACE_S and len(hist) > 5
                and abs(pos - hist[0][1]) < stall_min):
            print(f"    {label}: STALLED after {(pos-p0)/cpjd:+.3f} deg "
                  f"({abs(pos-hist[0][1])} counts in {now-hist[0][0]:.1f}s)", flush=True)
            send(bus, can_id, [0xF7])
            return pos, "stalled"
        if abs(pos - p0) >= pulses * ENCODER_CPR / MICROSTEPS_PER_REV - 300:
            s_last, s_still = pos, 0
            while time.time() - t0 < expected_s + 25:
                time.sleep(0.2)
                now = query(bus, can_id, 0x31)
                if now is None:
                    continue
                if abs(now - s_last) < SETTLE_COUNTS:
                    s_still += 1
                    if s_still >= SETTLE_SAMPLES:
                        break
                else:
                    s_still = 0
                s_last = now
            end = query(bus, can_id, 0x31) or pos
            print(f"    {label}: {(end-p0)/cpjd:+.3f} deg", flush=True)
            return end, None
    return query(bus, can_id, 0x31), "timed out"


def main():
    ap = argparse.ArgumentParser(description="Move each joint out and back; report the return error")
    ap.add_argument("degrees", type=float, help="distance each way")
    ap.add_argument("--joints", default=None, help="comma-separated (default: all that answer)")
    ap.add_argument("--speed", type=int, default=25)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    wanted = ([j.strip().lower() for j in args.joints.split(",")]
              if args.joints else sorted(JOINTS))
    for j in wanted:
        if j not in JOINTS:
            sys.exit(f"unknown joint {j!r}; choose from {sorted(JOINTS)}")

    print(f"round trip: {args.degrees} deg clockwise then back, joints "
          f"{', '.join(j.upper() for j in wanted)}\n")

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    results = []
    try:
        for j in wanted:
            can_id, gear, board, cw, remapped, max_speed = JOINTS[j]
            if cw is None:
                results.append((j, None, "no direction mapping -- skipped"))
                print(f"{j.upper()}: no confirmed direction mapping, skipped")
                continue
            speed = min(args.speed, max_speed) if max_speed else args.speed
            if query(bus, can_id, 0x31) is None:
                results.append((j, None, "not on the bus -- skipped"))
                print(f"{j.upper()}: no reply from 0x{can_id:02X}, skipped")
                continue
            if args.dry_run:
                print(f"{j.upper()}: would move {args.degrees} deg at speed {speed}")
                continue

            mask = io_mask(board, remapped)
            cpjd = gear * ENCODER_CPR / 360.0
            print(f"{j.upper()}  (0x{can_id:02X}, {gear}:1, speed {speed})", flush=True)
            start = query(bus, can_id, 0x31)
            send(bus, can_id, [0xF3, 0x01])
            time.sleep(0.3)

            _, note_a = leg(bus, can_id, gear, mask, args.degrees, cw, speed, "out ")
            time.sleep(0.6)
            back, note_b = leg(bus, can_id, gear, mask, args.degrees, cw ^ 0x80, speed, "back")

            send(bus, can_id, [0xF7])
            time.sleep(0.1)
            send(bus, can_id, [0xF3, 0x00])

            if back is None or start is None:
                results.append((j, None, "lost contact"))
                continue
            err = (back - start) / cpjd
            note = "; ".join(n for n in (note_a, note_b) if n) or None
            results.append((j, err, note))
            print(f"    return error {back - start:+d} counts = {err:+.4f} deg\n", flush=True)
    finally:
        print("=== summary ===")
        for j, err, note in results:
            if err is None:
                print(f"  {j.upper()}: {note}")
            elif note:
                print(f"  {j.upper()}: {err:+.4f} deg  -- {note}, so this is not a "
                      f"full round trip")
            else:
                print(f"  {j.upper()}: {err:+.4f} deg")
        print("\n  Return error is the figure that matters. A leg that hit an endstop or")
        print("  stalled did not travel the full distance, so its error is not comparable.")
        print("  None of this confirms the arm moved -- the encoder is on the motor shaft.")
        bus.shutdown()


if __name__ == "__main__":
    main()
