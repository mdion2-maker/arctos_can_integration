#!/usr/bin/env python3
"""
joint_endstop_test.py

Read-only live monitor for a joint's endstop / limit-switch inputs.

Sends only READ_IO (0x34), which reports the driver's input pin states as a
single byte. It never enables coils and never commands motion, so it is safe to
run at any time, including on an assembled arm.

Why this exists. The C++ driver decodes those bits like this:

    MKS_42D: left = bit 2 (IN_2), right = bit 3 (IN_1)
    MKS_57D: left = bit 1 inverted, right = bit 0 inverted

and the code carrying that mapping is annotated "TODO: Fix the logic". The two
hardware types disagree about both which bits to read and whether the signal is
active high or active low, and nothing in this project has confirmed either
against a real magnet. Rather than trust it, this script watches the whole byte
while you trigger each endstop by hand and tells you which bit actually moved
and in which direction. That answers three questions at once: whether the
endstops are wired to the driver's inputs at all, which bit each one is on, and
whether it reads high or low when triggered.

Usage:
    python3 joint_endstop_test.py --can-id 0x05
    python3 joint_endstop_test.py --can-id 0x05 --hz 20 --seconds 120

Trigger each endstop in turn with a magnet, or move the joint by hand onto it,
and watch which bit flips. Ctrl+C to stop and print the summary.
"""
import argparse
import sys
import time

import can

READ_IO = 0x34
READ_ENCODER = 0x31


def checksum(motor_id, data):
    return (motor_id + sum(data)) & 0xFF


def send_frame(bus, motor_id, data):
    bus.send(can.Message(arbitration_id=motor_id,
                         data=data + [checksum(motor_id, data)],
                         is_extended_id=False))


def query(bus, motor_id, opcode, timeout=0.4):
    while bus.recv(timeout=0.0) is not None:
        pass
    send_frame(bus, motor_id, [opcode])
    end = time.time() + timeout
    while time.time() < end:
        r = bus.recv(timeout=max(0.0, end - time.time()))
        if r is None:
            continue
        if r.arbitration_id == motor_id and len(r.data) >= 2 and r.data[0] == opcode:
            return bytes(r.data)
    return None


def bits(byte):
    """Bit 7 on the left, bit 0 on the right, spaced for reading."""
    return " ".join(f"{(byte >> i) & 1}" for i in range(7, -1, -1))


def main():
    ap = argparse.ArgumentParser(description="Read-only endstop / limit switch monitor")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--hz", type=float, default=10.0, help="Poll rate")
    ap.add_argument("--seconds", type=float, default=120.0, help="How long to watch")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if args.dry_run:
        f = [READ_IO, checksum(args.can_id, [READ_IO])]
        print("DRY RUN -- no bus opened, nothing sent.", flush=True)
        print(f"  poll frame -> cansend {args.channel} "
              f"{args.can_id:03X}#{f[0]:02X}{f[1]:02X}")
        print(f"  read-only: only 0x34 is ever sent. No coils, no motion.", flush=True)
        return

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    try:
        probe = query(bus, args.can_id, READ_IO)
        if probe is None:
            enc = query(bus, args.can_id, READ_ENCODER)
            if enc is None:
                sys.exit(f"ERROR: no response at all from 0x{args.can_id:02X}. "
                         f"Is the driver powered and on the bus?")
            sys.exit(f"ERROR: the driver answers 0x31 but not 0x34 (READ_IO). "
                     f"This firmware may not support the IO query.")

        baseline = probe[1]
        print(f"joint 0x{args.can_id:02X}  polling READ_IO (0x34) at {args.hz:g} Hz\n", flush=True)
        print(f"  bit number   7 6 5 4 3 2 1 0", flush=True)
        print(f"  at rest      {bits(baseline)}   (0x{baseline:02X})\n", flush=True)
        print("Trigger each endstop in turn. Bits that differ from rest are marked.", flush=True)
        print("Ctrl+C to stop.\n", flush=True)

        seen_high = 0        # bits observed as 1 at any point
        seen_low = 0xFF      # bits observed as 0 at any point (start all, clear as seen high)
        changed = 0
        last_shown = None
        deadline = time.time() + args.seconds
        period = 1.0 / args.hz

        while time.time() < deadline:
            t0 = time.time()
            resp = query(bus, args.can_id, READ_IO)
            if resp is not None:
                io = resp[1]
                seen_high |= io
                seen_low &= io          # bits still 1 here were never seen low
                changed |= (io ^ baseline)
                if io != last_shown:
                    marks = "".join(
                        (" *" if (io ^ baseline) >> i & 1 else "  ")
                        for i in range(7, -1, -1)
                    )
                    print(f"  {bits(io)}   (0x{io:02X}){marks}", flush=True)
                    last_shown = io
            time.sleep(max(0.0, period - (time.time() - t0)))

    except KeyboardInterrupt:
        print("\n(stopped)", flush=True)
    finally:
        print("\n=== summary ===", flush=True)
        try:
            print(f"    at rest        : {bits(baseline)}  (0x{baseline:02X})", flush=True)
            if changed:
                print(f"    bits that moved: {bits(changed)}", flush=True)
                for i in range(7, -1, -1):
                    if (changed >> i) & 1:
                        rest = (baseline >> i) & 1
                        print(f"      bit {i}: rest={rest} -> triggered={1 - rest}"
                              f"   ({'active low' if rest else 'active high'})")
                print("\n    Record these against the endstop you triggered. If a bit", flush=True)
                print("    disagrees with the C++ decode, the C++ is what is wrong.", flush=True)
            else:
                print("    bits that moved: none", flush=True)
                print("\n    No input changed while you triggered the endstops. Either", flush=True)
                print("    they are not wired to this driver's IN_1 / IN_2 pins, the", flush=True)
                print("    magnet never got close enough, or this joint has none.", flush=True)
        except NameError:
            pass
        bus.shutdown()


if __name__ == "__main__":
    main()
