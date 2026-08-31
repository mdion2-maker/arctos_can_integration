#!/usr/bin/env python3
"""
joint_reliability_test.py

Read-only reliability / telemetry test for a single Arctos joint over CAN.

Generic across joints -- pass --can-id and --gear-ratio for whichever joint
is physically wired up right now. Originally written while bringing up
Joint X (motor_id 0x01, MKS SERVO57D, 13.5:1) and reused as-is for Joint C
(motor_id 6 / CAN ID 0x06 per arctos_controller.yaml, MKS SERVO42D, NEMA17
same family as Joint B, 67.82:1) -- confirm the real CAN ID with
can_id_scan.py first, since DIP switches and config-file assumptions have
both been wrong before on this project.

This script ONLY sends read commands (0x31 READ_ENCODER, 0x39 READ_ERROR,
0x3A READ_ENABLE_STATE). It never sends 0xF3 (enable) or any position/speed
command, so the motor coils are never energized and the arm cannot move as a
result of running this script.

Usage:
    python3 joint_reliability_test.py --can-id 0x06 --gear-ratio 67.82 --samples 200
"""
import argparse
import statistics
import time

import can

READ_ENCODER = 0x31
READ_ERROR = 0x39
READ_ENABLE_STATE = 0x3A

ENCODER_CPR = 16384


def checksum(motor_id, data_bytes):
    return (motor_id + sum(data_bytes)) & 0xFF


def send_frame(bus, motor_id, data_bytes):
    crc = checksum(motor_id, data_bytes)
    frame = data_bytes + [crc]
    msg = can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False)
    bus.send(msg)


def query(bus, motor_id, opcode, expect_len, timeout=0.5):
    while bus.recv(timeout=0.0) is not None:
        pass
    t_sent = time.monotonic()
    send_frame(bus, motor_id, [opcode])
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        resp = bus.recv(timeout=max(0.0, end - time.monotonic()))
        if resp is None:
            continue
        if resp.arbitration_id != motor_id or len(resp.data) < expect_len:
            continue
        if resp.data[0] != opcode:
            continue
        latency = time.monotonic() - t_sent
        expected_crc = checksum(motor_id, list(resp.data[:-1]))
        crc_ok = resp.data[-1] == expected_crc
        return resp.data, latency, crc_ok
    return None, None, None


def read_encoder(bus, motor_id, gear_ratio, timeout=0.5):
    data, latency, crc_ok = query(bus, motor_id, READ_ENCODER, 8, timeout)
    if data is None:
        return None
    raw48 = int.from_bytes(data[1:7], byteorder="big", signed=True)
    motor_shaft_deg = raw48 * 360.0 / ENCODER_CPR
    joint_deg = motor_shaft_deg / gear_ratio
    return {"raw48": raw48, "motor_deg": motor_shaft_deg, "joint_deg": joint_deg,
            "latency_s": latency, "crc_ok": crc_ok}


def read_error(bus, motor_id, timeout=0.5):
    data, latency, crc_ok = query(bus, motor_id, READ_ERROR, 3, timeout)
    if data is None:
        return None
    return {"raw": data[1], "latency_s": latency, "crc_ok": crc_ok}


def read_enable_state(bus, motor_id, timeout=0.5):
    data, latency, crc_ok = query(bus, motor_id, READ_ENABLE_STATE, 3, timeout)
    if data is None:
        return None
    return {"enabled": bool(data[1]), "latency_s": latency, "crc_ok": crc_ok}


def main():
    parser = argparse.ArgumentParser(description="Read-only joint reliability test")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), required=True,
                         help="Joint's CAN ID, confirmed via can_id_scan.py -- do not guess")
    parser.add_argument("--gear-ratio", type=float, required=True,
                         help="Joint gear ratio, e.g. 13.5 for Joint X, 67.82 for Joint B/C")
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--period", type=float, default=0.05, help="Seconds between samples")
    parser.add_argument("--timeout", type=float, default=0.5, help="Per-query timeout (s)")
    args = parser.parse_args()

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")

    print(f"Joint reliability test -- CAN ID 0x{args.can_id:02X} on {args.channel} "
          f"(gear ratio {args.gear_ratio}:1)")
    print(f"Requesting {args.samples} encoder/error/enable-state samples, "
          f"read-only (no motor enable, no motion commands)\n")

    try:
        err = read_error(bus, args.can_id, args.timeout)
        en = read_enable_state(bus, args.can_id, args.timeout)
        if err is None or en is None:
            print("FAILED preliminary checks: could not reach the driver at "
                  f"0x{args.can_id:02X}. Run can_id_scan.py to confirm the real "
                  "CAN ID before continuing.")
            return
        print(f"Pre-test status: error_code={err['raw']}  coils_enabled={en['enabled']}")
        if en["enabled"]:
            print("NOTE: coils are already enabled by something else -- this test still "
                  "only reads, but be aware the joint could be holding torque right now.\n")

        joint_degs = []
        raw48s = []
        latencies = []
        successes = 0
        crc_failures = 0

        for i in range(args.samples):
            sample = read_encoder(bus, args.can_id, args.gear_ratio, args.timeout)
            if sample is None:
                print(f"  [{i+1}/{args.samples}] TIMEOUT -- no encoder response")
            else:
                successes += 1
                if not sample["crc_ok"]:
                    crc_failures += 1
                joint_degs.append(sample["joint_deg"])
                raw48s.append(sample["raw48"])
                latencies.append(sample["latency_s"])
                if (i + 1) % 20 == 0 or i == 0:
                    print(f"  [{i+1}/{args.samples}] raw48={sample['raw48']:>10}  "
                          f"joint_deg={sample['joint_deg']:.4f}  "
                          f"latency={sample['latency_s']*1000:.1f}ms  "
                          f"crc={'OK' if sample['crc_ok'] else 'BAD'}")
            time.sleep(args.period)

        print("\n--- Reliability Report ---")
        response_rate = 100.0 * successes / args.samples
        print(f"Response rate: {successes}/{args.samples} ({response_rate:.1f}%)")
        print(f"CRC failures among responses: {crc_failures}")
        if latencies:
            print(f"Round-trip latency: mean={statistics.mean(latencies)*1000:.1f}ms "
                  f"stdev={statistics.pstdev(latencies)*1000:.2f}ms "
                  f"max={max(latencies)*1000:.1f}ms")
        spread_counts = None
        if len(raw48s) >= 2:
            spread_counts = max(raw48s) - min(raw48s)
            spread_deg = spread_counts * 360.0 / ENCODER_CPR / args.gear_ratio
            print(f"Encoder jitter while stationary: {spread_counts} raw counts "
                  f"({spread_deg:.5f} joint degrees) peak-to-peak across {len(raw48s)} samples")
            print(f"Mean joint position: {statistics.mean(joint_degs):.5f} deg  "
                  f"stdev: {statistics.pstdev(joint_degs):.5f} deg")

        print("\nInterpretation:")
        if response_rate < 95.0:
            print("  Response rate below 95% -- suspect wiring/termination, bitrate mismatch, "
                  "or a marginal ground connection before trusting this link for motion.")
        elif crc_failures > 0:
            print("  Responses arrived but some failed checksum -- treat as electrically noisy, "
                  "investigate CAN_H/CAN_L wiring and termination resistors before moving.")
        elif spread_counts is not None and spread_counts > 50:
            print("  Encoder reading drifted noticeably while the joint should be stationary -- "
                  "check nothing is nudging the shaft and that the magnetic encoder is seated well.")
        else:
            print("  Link looks solid: high response rate, clean checksums, stable encoder. "
                  "Safe to move on to the supervised tiny-move calibration test.")

    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
