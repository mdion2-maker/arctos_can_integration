#!/usr/bin/env python3
"""
joint_b_micro_move_test_v2.py

Same safe small (~0.1 deg of Joint B) test move as before, but now reading
the encoder with the CORRECT format confirmed from can_protocol.cpp and
verified against your own hand-rotation test:

    raw48 = signed 48-bit big-endian integer over response bytes 1-6
    motor_shaft_degrees = raw48 * 360.0 / 16384.0

(The old carry+value split used in the first version of this script was
wrong -- this replaces it.)

The WRITE side (0xF5 frame construction, byte order, checksum) was already
confirmed correct from motor_driver.cpp and is unchanged here.
"""
import can
import time

CAN_ID = 0x05
CHANNEL = "can0"

TEST_DELTA_COUNTS = 300      # ~0.1 degree of Joint B (67.82:1 gear, 16384 cpr)
SPEED = 20
ACCEL = 5
GEAR_RATIO = 67.82
COUNTS_PER_JOINT_DEGREE = GEAR_RATIO * 16384 / 360.0   # ~3086.56


def checksum(motor_id, data_bytes):
    return (motor_id + sum(data_bytes)) & 0xFF


def send_frame(bus, motor_id, data_bytes):
    crc = checksum(motor_id, data_bytes)
    frame = data_bytes + [crc]
    msg = can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False)
    bus.send(msg)
    return frame


def read_encoder_motor_shaft_degrees(bus, motor_id, timeout=1.0):
    """Uses 0x31 (READ_ENCODER), the opcode the real driver actually uses,
    with the confirmed decodeInt48 format."""
    send_frame(bus, motor_id, [0x31])
    end = time.time() + timeout
    while time.time() < end:
        resp = bus.recv(timeout=end - time.time())
        if resp is None:
            break
        if resp.arbitration_id == motor_id and len(resp.data) >= 7 and resp.data[0] == 0x31:
            raw48 = int.from_bytes(resp.data[1:7], byteorder="big", signed=True)
            motor_shaft_degrees = raw48 * 360.0 / 16384.0
            return raw48, motor_shaft_degrees
    raise TimeoutError("No encoder response received")


def enable_motor(bus, motor_id):
    send_frame(bus, motor_id, [0xF3, 0x01]); time.sleep(0.2)


def disable_motor(bus, motor_id):
    send_frame(bus, motor_id, [0xF3, 0x00]); time.sleep(0.2)


def move_small(bus, motor_id, delta_counts, speed, accel):
    """Confirmed frame layout from motor_driver.cpp: high-mid-low position bytes."""
    speed_hi, speed_lo = (speed >> 8) & 0xFF, speed & 0xFF
    pos = delta_counts & 0xFFFFFF
    pos_hi, pos_mid, pos_lo = (pos >> 16) & 0xFF, (pos >> 8) & 0xFF, pos & 0xFF
    data = [0xF5, speed_hi, speed_lo, accel, pos_hi, pos_mid, pos_lo]
    return send_frame(bus, motor_id, data)


def main():
    bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    try:
        raw0, deg0 = read_encoder_motor_shaft_degrees(bus, CAN_ID)
        print(f"Start: raw48={raw0}  motor-shaft degrees={deg0:.4f}  "
              f"joint-B degrees={deg0/GEAR_RATIO:.4f}")

        input("Enable motor and send a ~0.1 degree (Joint B) test move? "
              "Press Enter to continue, Ctrl+C to abort...")

        enable_motor(bus, CAN_ID)
        frame = move_small(bus, CAN_ID, TEST_DELTA_COUNTS, SPEED, ACCEL)
        print(f"Sent frame: {[hex(b) for b in frame]}")
        time.sleep(2.0)

        raw1, deg1 = read_encoder_motor_shaft_degrees(bus, CAN_ID)
        print(f"End:   raw48={raw1}  motor-shaft degrees={deg1:.4f}  "
              f"joint-B degrees={deg1/GEAR_RATIO:.4f}")

        moved_raw = raw1 - raw0
        moved_joint_deg = moved_raw / COUNTS_PER_JOINT_DEGREE
        print(f"\nObserved change: {moved_raw} raw counts "
              f"({moved_joint_deg:.4f} deg of Joint B)")
        print(f"Commanded: {TEST_DELTA_COUNTS} raw counts "
              f"({TEST_DELTA_COUNTS/COUNTS_PER_JOINT_DEGREE:.4f} deg of Joint B)")
        print("If observed is close to commanded (allowing for the 'inverted' "
              "sign flag), the write path is fully validated end to end.")
    finally:
        print("Disabling motor.")
        disable_motor(bus, CAN_ID)
        bus.shutdown()


if __name__ == "__main__":
    main()
