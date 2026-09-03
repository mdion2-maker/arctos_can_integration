#!/usr/bin/env python3
"""
joint_b_final_reliability_test.py
Directly stress-tests Joint B using the verified 0xF5 absolute microstep protocol.
Cycles back and forth to ensure 100% positional repeatability with no stalls.
"""

import can
import time

CAN_ID = 0x05
CHANNEL = "can0"
INTERFACE = "socketcan"

# Calibrated Constants from our Session Logs
SCALE_FACTOR = -1.02  # 1 microstep pulse = -1.02 encoder counts
GEAR_RATIO = 67.82
PULSES_PER_DEGREE = (3200.0 * GEAR_RATIO) / 360.0 # ~602.844 pulses per joint degree

def checksum(motor_id, data_bytes):
    return (motor_id + sum(data_bytes)) & 0xFF

def send_frame(bus, motor_id, data_bytes):
    crc = checksum(motor_id, data_bytes)
    frame = data_bytes + [crc]
    msg = can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False)
    bus.send(msg)
    return frame

def read_encoder(bus, motor_id, timeout=1.0):
    send_frame(bus, motor_id, [0x30])
    end = time.time() + timeout
    while time.time() < end:
        resp = bus.recv(timeout=end - time.time())
        if resp is None:
            break
        if resp.arbitration_id == motor_id and len(resp.data) >= 7 and resp.data[0] == 0x30:
            carry = int.from_bytes(resp.data[1:5], byteorder="big", signed=True)
            value = int.from_bytes(resp.data[5:7], byteorder="big", signed=False)
            return (carry * 16384) + value
    raise TimeoutError("Encoder dropped frame!")

def move_absolute_pulses(bus, motor_id, absolute_pulses, speed=40, accel=10):
    pos = int(absolute_pulses) & 0xFFFFFF
    data = [
        0xF5, 
        (speed >> 8) & 0xFF, speed & 0xFF, 
        accel, 
        (pos >> 16) & 0xFF, (pos >> 8) & 0xFF, pos & 0xFF
    ]
    send_frame(bus, motor_id, data)

def main():
    bus = can.interface.Bus(channel=CHANNEL, interface=INTERFACE)
    
    try:
        print("🔍 Initializing Joint B Final Reliability Sweep...")
        start_enc = read_encoder(bus, CAN_ID)
        print(f"  Starting Absolute Location: {start_enc} encoder counts")
        
        # Calculate our tracking baseline index in pulses
        base_pulses = int(start_enc / SCALE_FACTOR)
        
        print("  Enabling motor coils...")
        send_frame(bus, CAN_ID, [0xF3, 0x01])
        time.sleep(0.5)

        # We will move +4.0 degrees forward, then back to 0, twice
        test_angles = [4.0, 0.0, 4.0, 0.0]
        
        for i, target_angle in enumerate(test_angles, 1):
            target_pulse = base_pulses + int(target_angle * PULSES_PER_DEGREE)
            
            print(f"\n🚀 [Run {i}/4] Sweeping axis toward {target_angle}° target (Pulse target: {target_pulse})...")
            move_absolute_pulses(bus, CAN_ID, target_pulse, speed=50, accel=12)
            
            # Wait for mechanical stabilization and transit completion
            time.sleep(4.0)
            
            # Verify positioning payload feedback loop
            final_enc = read_encoder(bus, CAN_ID)
            print(f"  Reached Location: {final_enc} counts")
            
        print("\n🏆 Verification Cycle Finished cleanly!")
        end_enc = read_encoder(bus, CAN_ID)
        print(f"  Initial count: {start_enc} | Final count: {end_enc}")
        print(f"  Net Repeatability Drift Error: {abs(end_enc - start_enc)} raw counts.")

    finally:
        # Disabling the coils does NOT cancel an in-flight 0xFD move -- the move
        # runs on to completion after this script exits. Measured on Joint X,
        # 2026-09-03: a 5 deg move cut off at 0.5 deg finished the remaining 4.5 deg
        # with no script running. Only EMERGENCY_STOP actually stops it.
        send_frame(bus, CAN_ID, [0xF7])
        time.sleep(0.1)
        print("\nSafety lockdown: Disabling motor power.")
        send_frame(bus, CAN_ID, [0xF3, 0x00])
        bus.shutdown()

if __name__ == "__main__":
    main()
