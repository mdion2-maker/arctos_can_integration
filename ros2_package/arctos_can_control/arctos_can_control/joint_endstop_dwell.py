#!/usr/bin/env python3
"""
joint_endstop_dwell.py

Measures how long each endstop trigger actually holds.

joint_endstop_test.py answers "does the input move at all". Once it does, the
next question is whether a trigger is *solid*. That script cannot answer it: it
prints only on change and stamps no times, so a two-second hold and a 50 ms tap
produce byte-for-byte identical output. A limit that chatters still looks like a
working limit there, and only reveals itself mid-homing as the driver watching
the endstop open and close as the joint creeps up on it.

So this one timestamps every edge and reports, per channel, how long each
trigger held and how far apart the events were. A magnet held deliberately for a
slow count of three should produce one event of roughly that length. Several
short events instead means the magnet is sitting at the edge of the sensor's
range and the output is bouncing across the threshold -- fix that by moving the
sensor or the magnet, not in software.

Read-only. Sends only READ_IO (0x34). Never enables coils, never commands
motion.

Bit mapping, as measured on Joint C (0x06) with limit port remap enabled:

    bit 0  IN_1 -> En   pin
    bit 1  IN_2 -> Dir  pin   (only present once remap is on)

Both idle high and read low when triggered.

Usage:
    python3 joint_endstop_dwell.py --can-id 0x06
    python3 joint_endstop_dwell.py --can-id 0x06 --hz 20 --seconds 180
"""
import argparse
import sys
import time

import can

READ_IO = 0x34

# Two events closer together than this look like bounce rather than two
# deliberate passes by hand.
CHATTER_GAP_S = 0.25
# A hold at least this long is what a deliberate trigger should look like.
SOLID_HOLD_S = 1.0

CHANNELS = [
    (0, "IN_1 -> En "),
    (1, "IN_2 -> Dir"),
]


def checksum(motor_id, data):
    return (motor_id + sum(data)) & 0xFF


def query(bus, motor_id, opcode, timeout=0.3):
    while bus.recv(timeout=0.0) is not None:
        pass
    bus.send(can.Message(arbitration_id=motor_id,
                         data=[opcode, checksum(motor_id, [opcode])],
                         is_extended_id=False))
    end = time.time() + timeout
    while time.time() < end:
        r = bus.recv(timeout=max(0.0, end - time.time()))
        # The driver's reply to 0x34 is DLC 3: [34, status, crc]. A DLC 2 frame
        # carrying the same opcode is somebody's *query*, not an answer -- which
        # happens whenever another tool is polling can0 at the same time.
        if (r is not None and r.arbitration_id == motor_id
                and len(r.data) == 3 and r.data[0] == opcode):
            return r.data[1]
    return None


def bits(byte):
    return " ".join(f"{(byte >> i) & 1}" for i in range(7, -1, -1))


def summarise(name, events, resolution):
    """events is a list of (start_offset, duration) for one channel."""
    print(f"\n  {name}", flush=True)
    if not events:
        print("      never triggered", flush=True)
        return

    holds = [d for _s, d in events]
    longest = max(holds)
    print(f"      {len(events)} trigger(s), longest held {longest:.2f}s", flush=True)
    for start, dur in events:
        flag = "  <-- very short" if dur < resolution * 3 else ""
        print(f"        at t={start:6.2f}s  held {dur:5.2f}s{flag}", flush=True)

    gaps = [events[i][0] - (events[i - 1][0] + events[i - 1][1])
            for i in range(1, len(events))]
    bouncy = [g for g in gaps if g < CHATTER_GAP_S]

    print(flush=True)
    if longest >= SOLID_HOLD_S and not bouncy:
        print("      SOLID. A deliberate hold registered as one clean event.", flush=True)
    elif bouncy:
        print(f"      CHATTER. {len(bouncy)} event(s) began within "
              f"{CHATTER_GAP_S}s of the previous one ending.", flush=True)
        print("      The output is crossing its threshold repeatedly, which is what", flush=True)
        print("      a magnet sitting at the edge of range does. Move the sensor", flush=True)
        print("      closer to the magnet's path, or use a stronger magnet.", flush=True)
    else:
        print(f"      MARGINAL. Nothing held for {SOLID_HOLD_S}s or more. If you meant", flush=True)
        print("      to hold it, the trigger is dropping out on its own.", flush=True)


def main():
    ap = argparse.ArgumentParser(description="Measure endstop trigger dwell times")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--seconds", type=float, default=180.0)
    args = ap.parse_args()

    resolution = 1.0 / args.hz
    bus = can.interface.Bus(channel=args.channel, interface="socketcan")

    events = {bit: [] for bit, _ in CHANNELS}
    open_since = {bit: None for bit, _ in CHANNELS}
    t0 = time.time()

    try:
        baseline = query(bus, args.can_id, READ_IO)
        if baseline is None:
            sys.exit(f"ERROR: no reply to 0x34 from 0x{args.can_id:02X}.")

        print(f"joint 0x{args.can_id:02X}  dwell timing at {args.hz:g} Hz "
              f"(edges shorter than {resolution * 1000:.0f} ms are invisible)\n", flush=True)
        print(f"  at rest   {bits(baseline)}  (0x{baseline:02X})\n", flush=True)
        print("Hold each pole on the sensor for a slow count of three, then remove it.", flush=True)
        print("Ctrl+C to stop early.\n", flush=True)

        prev = baseline
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            loop_start = time.time()
            io = query(bus, args.can_id, READ_IO)
            if io is not None:
                now = time.time() - t0
                for bit, name in CHANNELS:
                    was_low = not (prev >> bit) & 1
                    is_low = not (io >> bit) & 1
                    if is_low and not was_low:
                        open_since[bit] = now
                        print(f"  t={now:6.2f}s  {name}  TRIGGERED", flush=True)
                    elif was_low and not is_low and open_since[bit] is not None:
                        dur = now - open_since[bit]
                        events[bit].append((open_since[bit], dur))
                        open_since[bit] = None
                        print(f"  t={now:6.2f}s  {name}  released after {dur:.2f}s", flush=True)
                prev = io
            time.sleep(max(0.0, resolution - (time.time() - loop_start)))

    except KeyboardInterrupt:
        print("\n(stopped)", flush=True)
    finally:
        # A trigger still held when time ran out is still worth counting.
        end_t = time.time() - t0
        for bit, _name in CHANNELS:
            if open_since[bit] is not None:
                events[bit].append((open_since[bit], end_t - open_since[bit]))

        print("\n=== dwell summary ===", flush=True)
        for bit, name in CHANNELS:
            summarise(name, events[bit], resolution)
        bus.shutdown()


if __name__ == "__main__":
    main()
