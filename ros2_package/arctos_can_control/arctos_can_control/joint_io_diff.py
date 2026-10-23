#!/usr/bin/env python3
"""
joint_io_diff.py

Finds where a driver reports an endstop, without assuming it is READ_IO.

joint_endstop_test.py watches READ_IO (0x34) because that is what the project's
C++ decodes limit switches from. If an endstop is verifiably triggering -- the
sensor's own LED lights -- and 0x34 never moves, the assumption itself is the
thing to test. This script polls every documented read-only command with the
magnet away, then again with it held on, and reports any byte or bit that
differs.

Read-only. Only query opcodes are sent, never a write, never a motion command,
never an enable. The opcode list deliberately excludes 0x3D
(RELEASE_SHAFT_PROTECTION) and everything undocumented, because an unknown
opcode may well be a write.

Position-ish registers (0x31, 0x35, 0x39) will wander if the shaft moves, so
each phase is sampled several times and only values that are STABLE within both
phases but DIFFERENT between them are reported. That filters encoder noise and
leaves genuine state changes.

Usage:
    python3 joint_io_diff.py --can-id 0x06

It will prompt you to hold the magnet on the sensor part way through.
"""
import argparse
import sys
import time

import can

# Documented read-only queries only.
OPCODES = [
    (0x30, "READ_ENCODER_CARRY"),
    (0x31, "READ_ENCODER"),
    (0x32, "READ_VELOCITY"),
    (0x33, "READ_PULSES"),
    (0x34, "READ_IO"),
    (0x35, "READ_RAW_ENCODER"),
    (0x39, "READ_ERROR"),
    (0x3A, "READ_ENABLE_STATE"),
    (0x3E, "READ_SHAFT_PROTECTION"),
    (0xF1, "QUERY_MOTOR"),
]


def checksum(motor_id, data):
    return (motor_id + sum(data)) & 0xFF


def query(bus, motor_id, opcode, timeout=0.35):
    while bus.recv(timeout=0.0) is not None:
        pass
    bus.send(can.Message(arbitration_id=motor_id,
                         data=[opcode, checksum(motor_id, [opcode])],
                         is_extended_id=False))
    end = time.time() + timeout
    while time.time() < end:
        r = bus.recv(timeout=max(0.0, end - time.time()))
        if r and r.arbitration_id == motor_id and len(r.data) >= 2 and r.data[0] == opcode:
            return tuple(r.data)
    return None


def sample_all(bus, motor_id, rounds=4):
    """Returns {opcode: set_of_responses}. A stable register gives one entry."""
    seen = {op: set() for op, _ in OPCODES}
    for _ in range(rounds):
        for op, _name in OPCODES:
            r = query(bus, motor_id, op)
            if r is not None:
                seen[op].add(r)
        time.sleep(0.05)
    return seen


def describe(frame):
    return " ".join(f"{b:02X}" for b in frame)


def main():
    ap = argparse.ArgumentParser(description="Find which register reports the endstop")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--rounds", type=int, default=4)
    args = ap.parse_args()

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    try:
        print("\nPhase 1 of 2. Keep the magnet AWAY from the sensor.")
        input("Press Enter when ready... ")
        off = sample_all(bus, args.can_id, args.rounds)

        responded = [f"0x{op:02X}" for op, _ in OPCODES if off[op]]
        silent = [f"0x{op:02X}" for op, _ in OPCODES if not off[op]]
        print(f"  answered: {', '.join(responded) if responded else 'nothing'}")
        if silent:
            print(f"  no reply: {', '.join(silent)}")

        print("\nPhase 2 of 2. Hold the magnet ON the sensor, so its LED is lit,")
        print("and keep it there.")
        input("Press Enter while holding it... ")
        on = sample_all(bus, args.can_id, args.rounds)

        print("\n=== differences ===")
        found = False
        for op, name in OPCODES:
            a, b = off[op], on[op]
            if not a or not b:
                continue
            # Only trust registers that held one value throughout each phase.
            if len(a) != 1 or len(b) != 1:
                continue
            fa, fb = next(iter(a)), next(iter(b))
            if fa == fb:
                continue
            found = True
            print(f"\n  0x{op:02X} {name}")
            print(f"    magnet off: {describe(fa)}")
            print(f"    magnet on : {describe(fb)}")
            for i in range(min(len(fa), len(fb))):
                if fa[i] != fb[i]:
                    diff = fa[i] ^ fb[i]
                    bitlist = [str(k) for k in range(7, -1, -1) if (diff >> k) & 1]
                    print(f"    byte {i}: 0x{fa[i]:02X} -> 0x{fb[i]:02X}"
                          f"   bit(s) {', '.join(bitlist)}")

        if not found:
            print("  Nothing changed in any register.")
            print("\n  The driver is not seeing the sensor at all. With the sensor's")
            print("  own LED confirmed lighting, that puts the fault between the")
            print("  sensor's output pin and the driver's input pin -- the signal")
            print("  wire, a missing pull-up, or the input not being enabled in the")
            print("  driver's own settings. No software change can recover this.")
        else:
            print("\n  Note which register moved. If it is not 0x34, the C++ driver")
            print("  is watching the wrong one.")

    except KeyboardInterrupt:
        print("\n(stopped)")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
