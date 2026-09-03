#!/usr/bin/env python3
"""
joint_micro_move_test.py

Supervised, tiny (~0.1 deg of joint travel) first-motion test for a single
Arctos joint. Generic across joints -- pass --can-id and --gear-ratio for
whichever joint is physically wired up right now.

DO NOT run this unattended. Run it yourself, standing next to the arm with
a hand on the power switch, after joint_reliability_test.py has shown a
clean link. If this is a joint's first-ever motion test, its coils have
never been energized before and it has no calibrated travel limits yet --
there is nothing in software stopping it from driving into a mechanical
hard stop if a sign/scale assumption below is wrong.

Protocol (confirmed against motor_types.hpp / can_protocol.cpp, the same
opcodes the real arctos_motor_driver C++ node uses):
    0x31 READ_ENCODER      -> 48-bit signed big-endian raw counts, 16384 counts/rev
    0xF3 ENABLE_MOTOR       -> [0xF3, 0x01]/[0xF3, 0x00]
    0xFD POSITION_CONTROL   -> [0xFD, direction|speed_hi_nibble, speed_lo, accel, pulses_hi, pulses_mid, pulses_lo]
    0xF7 EMERGENCY_STOP     -> [0xF7]

CAUTION -- do not switch this back to 0xF5 ABSOLUTE_POSITION with a raw
two's-complement-packed target. On 2026-08-31, that exact approach caused a
real incident on Joint B: after extensive earlier un-homed testing, its raw
encoder position had drifted to roughly -274,000 counts, and packing
`target_raw = raw0 + delta_counts` as `int(target_raw) & 0xFFFFFF` produced a
bit pattern the driver read as an enormous, wrong target (~8.1M pulses,
~178,000 degrees) -- a small 300-count nudge turned into a large,
uninterrupted, non-settling motion that had to be stopped by cutting motor
power. This script now uses POSITION_CONTROL (0xFD) instead, the same
direction-byte-plus-always-positive-magnitude pattern already proven safe
across every joint in this project via joint_statistical_reliability_test.py
-- it never depends on the current absolute position, so it cannot repeat
this failure no matter how far a joint's raw counter has drifted.

Every frame this script sends -- including EMERGENCY_STOP -- is checksummed
via send_frame()/checksum() below, matching motor_driver.cpp's sendFrame(),
which always appends a checksum byte. Do not manually cansend a bare "F7"
with no checksum -- that form does not match what the real driver actually
transmits (confirmed by reading motor_driver.cpp's stopMotor() and
can_protocol.cpp's sendFrame() directly); always compute and append the
checksum, as this script's own emergency_stop() does.

Joint hardware profiles confirmed from arctos_controller.yaml (NOT the
placeholder numbers in arctos_can/mks_driver.py, which were copied from an
unrelated joint's calibration and are not trustworthy):
    Joint X: motor_id 0x01, MKS SERVO57D, gear_ratio 13.5,  inverted=true
    Joint B: motor_id 0x05, MKS SERVO42D, gear_ratio 67.82, inverted=true
    Joint C: motor_id 0x06, MKS SERVO42D, gear_ratio 67.82, inverted=true
             (NOT 0x03 -- an older hardware map in this project's docs listed
             Joint C at 0x03, which is actually Z_joint's ID in a different,
             3-joint XYZ test config. arctos_controller.yaml, the real 6-joint
             arm config, defines C_joint's motor_id as 6. Confirm empirically
             with can_id_scan.py regardless -- don't trust either number blind.)

Usage:
    python3 joint_micro_move_test.py --can-id 0x06 --gear-ratio 67.82
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
MICROSTEPS_PER_MOTOR_REV = 3200.0  # 200 full steps/rev x 16 microsteps/step -- POSITION_CONTROL's
                                    # pulses field is in THIS scale, not ENCODER_CPR. Mixing the two
                                    # up once produced a 16384/3200 = 5.12x magnitude error here.
TEST_DELTA_COUNTS = 300  # small raw-encoder-scale move; re-check sign/magnitude live
SPEED = 15
ACCEL = 2


def checksum(motor_id, data_bytes):
    return (motor_id + sum(data_bytes)) & 0xFF


def send_frame(bus, motor_id, data_bytes):
    crc = checksum(motor_id, data_bytes)
    frame = data_bytes + [crc]
    msg = can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False)
    bus.send(msg)
    return frame


def read_encoder(bus, motor_id, timeout=1.0):
    send_frame(bus, motor_id, [READ_ENCODER])
    end = time.time() + timeout
    while time.time() < end:
        resp = bus.recv(timeout=end - time.time())
        if resp is None:
            break
        if resp.arbitration_id == motor_id and len(resp.data) >= 8 and resp.data[0] == READ_ENCODER:
            raw48 = int.from_bytes(resp.data[1:7], byteorder="big", signed=True)
            return raw48, raw48 * 360.0 / ENCODER_CPR
    raise TimeoutError("No encoder response -- re-run can_id_scan.py before trying again")


def enable_motor(bus, motor_id):
    send_frame(bus, motor_id, [ENABLE_MOTOR, 0x01])
    time.sleep(0.2)


def disable_motor(bus, motor_id):
    send_frame(bus, motor_id, [ENABLE_MOTOR, 0x00])
    time.sleep(0.2)


def emergency_stop(bus, motor_id):
    send_frame(bus, motor_id, [EMERGENCY_STOP])


def move_position_control(bus, motor_id, pulses, speed, accel, direction):
    """Always-positive magnitude + separate direction byte -- see the CAUTION
    in this file's module docstring for why this replaced an absolute-target
    encoding that broke once a joint's raw counter drifted far from zero."""
    pulses = int(pulses) & 0xFFFFFF
    data = [POSITION_CONTROL, direction + ((speed >> 8) & 0b1111), speed & 0xFF, accel,
            (pulses >> 16) & 0xFF, (pulses >> 8) & 0xFF, pulses & 0xFF]
    return send_frame(bus, motor_id, data)


def main():
    parser = argparse.ArgumentParser(description="Supervised joint first-motion test")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    parser.add_argument("--gear-ratio", type=float, required=True)
    parser.add_argument("--delta-counts", type=int, default=TEST_DELTA_COUNTS,
                         help="Raw encoder-scale delta to command (default ~300)")
    args = parser.parse_args()

    estop_checksum = checksum(args.can_id, [EMERGENCY_STOP])

    print("=" * 70)
    print("SUPERVISED JOINT FIRST-MOTION TEST -- read the header before continuing")
    print("Keep a second terminal open running: candump -tz can0")
    print(f"Emergency stop, any time: cansend can0 "
          f"{args.can_id:03X}#{EMERGENCY_STOP:02X}{estop_checksum:02X}")
    print("=" * 70)

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    try:
        raw0, deg0 = read_encoder(bus, args.can_id)
        print(f"\nStart position: raw48={raw0}  motor-shaft degrees={deg0:.4f}  "
              f"joint degrees={deg0/args.gear_ratio:.4f}")

        commanded_joint_deg = args.delta_counts * 360.0 / ENCODER_CPR / args.gear_ratio
        confirm = input(
            f"\nAbout to enable coils on 0x{args.can_id:02X} and command a "
            f"{args.delta_counts} raw-count move (~{commanded_joint_deg:.3f} "
            "joint degrees). Stand clear of pinch points. Type 'go' to proceed, "
            "anything else aborts: "
        )
        if confirm.strip().lower() != "go":
            print("Aborted -- no commands sent.")
            return

        enable_motor(bus, args.can_id)
        direction = 0x00 if args.delta_counts >= 0 else 0x80
        pulses = round(abs(args.delta_counts) * MICROSTEPS_PER_MOTOR_REV / ENCODER_CPR)
        frame = move_position_control(bus, args.can_id, pulses, SPEED, ACCEL, direction)
        print(f"Sent frame: {[hex(b) for b in frame]}")

        print("Watching position for 5s (Ctrl+C to emergency-stop immediately)...")
        try:
            for _ in range(10):
                time.sleep(0.5)
                raw_now, deg_now = read_encoder(bus, args.can_id)
                print(f"  raw48={raw_now:>10}  joint_deg={deg_now/args.gear_ratio:.4f}")
        except KeyboardInterrupt:
            print("\nCtrl+C -- sending EMERGENCY_STOP now")
            emergency_stop(bus, args.can_id)
            sys.exit(1)

        raw1, deg1 = read_encoder(bus, args.can_id)
        moved_raw = raw1 - raw0
        moved_joint_deg = moved_raw * 360.0 / ENCODER_CPR / args.gear_ratio
        print(f"\nObserved change: {moved_raw} raw counts ({moved_joint_deg:.4f} joint degrees)")
        print(f"Commanded change: {args.delta_counts} raw counts ({commanded_joint_deg:.4f} joint degrees)")
        print("If observed direction is opposite to commanded, this joint's 'inverted' flag "
              "(true for X/B/C in arctos_controller.yaml) already accounts for that in "
              "higher-level code -- this raw test does not apply it, so a sign flip here "
              "is expected and fine.")
        print("If the magnitude doesn't roughly match, do not proceed to larger moves until "
              "you understand why (wrong CAN ID, wrong microstep/subdivision setting, slipping "
              "coupler, etc).")

    finally:
        # Disabling the coils does NOT cancel an in-flight 0xFD move -- the move
        # runs on to completion after this script exits. Measured on Joint X,
        # 2026-09-03: a 5 deg move cut off at 0.5 deg finished the remaining 4.5 deg
        # with no script running. Only EMERGENCY_STOP actually stops it.
        emergency_stop(bus, args.can_id)
        time.sleep(0.1)
        print("\nDisabling motor coils.")
        disable_motor(bus, args.can_id)
        bus.shutdown()


if __name__ == "__main__":
    main()
