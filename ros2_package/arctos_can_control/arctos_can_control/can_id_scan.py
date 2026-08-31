#!/usr/bin/env python3
"""
can_id_scan.py

Read-only CAN-ID discovery tool for a single Arctos joint's MKS driver.

Generic across joints -- point it at whichever joint is physically wired up
right now. No config file or DIP-switch reading has been trustworthy enough
to skip this step: Joint X's DIP switches (2 and 3 ON) turned out to map to
nothing at all (that driver turned out to be an RS485 variant with no CAN
transceiver), and Joint C's assumed CAN ID has disagreed across this
project's own documents (0x03 in one hardware-map table, motor_id 6 i.e.
0x06 in the authoritative arctos_controller.yaml). Rather than trust either
source, this script sends the read-only 0x31 READ_ENCODER command (opcode
confirmed against motor_types.hpp / can_protocol.cpp) to a range of
candidate IDs and reports which one(s) actually answer.

This never enables motor coils and never commands motion - it only asks
"are you there and what's your position", so it is safe to run at any time
while can0 is up, uncalibrated or not.

Usage:
    python3 can_id_scan.py --channel can0 --ids 1-8
"""
import argparse
import time

import can

READ_ENCODER = 0x31


def checksum(motor_id, data_bytes):
    return (motor_id + sum(data_bytes)) & 0xFF


def send_frame(bus, motor_id, data_bytes):
    crc = checksum(motor_id, data_bytes)
    frame = data_bytes + [crc]
    msg = can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False)
    bus.send(msg)


def decode_int48(data6):
    raw = int.from_bytes(data6, byteorder="big", signed=True)
    return raw, raw * 360.0 / 16384.0


def probe_id(bus, motor_id, timeout=0.4):
    # Drain anything stale before probing.
    while bus.recv(timeout=0.0) is not None:
        pass

    send_frame(bus, motor_id, [READ_ENCODER])
    end = time.time() + timeout
    while time.time() < end:
        resp = bus.recv(timeout=max(0.0, end - time.time()))
        if resp is None:
            continue
        if resp.arbitration_id != motor_id:
            continue
        if len(resp.data) < 8 or resp.data[0] != READ_ENCODER:
            continue
        expected_crc = checksum(motor_id, list(resp.data[:-1]))
        crc_ok = resp.data[-1] == expected_crc
        raw48, degrees = decode_int48(resp.data[1:7])
        return {"raw48": raw48, "motor_shaft_degrees": degrees, "crc_ok": crc_ok}
    return None


def main():
    parser = argparse.ArgumentParser(description="Scan for a joint's real CAN ID")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--ids", default="1-8", help="Inclusive ID range, e.g. 1-8")
    parser.add_argument("--timeout", type=float, default=0.4, help="Per-ID response timeout (s)")
    args = parser.parse_args()

    lo, hi = (int(x) for x in args.ids.split("-"))
    candidate_ids = list(range(lo, hi + 1))

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    found = []
    try:
        print(f"Scanning CAN IDs {lo}-{hi} on {args.channel} (READ_ENCODER, read-only, no motion)...\n")
        for motor_id in candidate_ids:
            result = probe_id(bus, motor_id, timeout=args.timeout)
            tag = f"0x{motor_id:02X}"
            if result is None:
                print(f"  {tag}: no response")
                continue
            crc_flag = "OK" if result["crc_ok"] else "CRC MISMATCH"
            print(
                f"  {tag}: RESPONDED  raw48={result['raw48']:>10}  "
                f"motor-shaft degrees={result['motor_shaft_degrees']:.3f}  checksum={crc_flag}"
            )
            found.append(motor_id)
    finally:
        bus.shutdown()

    print()
    if not found:
        print("No motors responded. Check: can0 is up, wiring/termination, driver has power, "
              "and that the ID range covers your DIP switch setting.")
    elif len(found) == 1:
        print(f"Exactly one responder: 0x{found[0]:02X}. That is this joint's real CAN ID -- "
              f"use this value (not a config-file or DIP-switch assumption) in every script below.")
    else:
        ids_str = ", ".join(f"0x{i:02X}" for i in found)
        print(f"Multiple responders found ({ids_str}). If only one joint should be wired right "
              "now, investigate before proceeding -- this could mean stale bus traffic, another "
              "powered driver on the bus, or an address collision.")


if __name__ == "__main__":
    main()
