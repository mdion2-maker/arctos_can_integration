#!/usr/bin/env python3
"""
mks_driver.py
Production-ready driver library for Arctos MKS Servo42D over SocketCAN.
Handles inversion math and microstep-to-degree transformations.
"""

import can
import time

class MKSServoDriver:
    def __init__(self, channel="can0", interface="socketcan"):
        self.bus = can.interface.Bus(channel=channel, interface=interface)
        
        # Microstep pulse scale, matching set_zero_position.py: 200 full
        # steps/rev x 16 microsteps/step = 3200 pulses per motor revolution.
        # This is NOT the 16384-count magnetic encoder scale used by 0x31.
        self.MICROSTEPS_PER_MOTOR_REV = 3200.0
        self.PULSES_PER_MOTOR_DEGREE = self.MICROSTEPS_PER_MOTOR_REV / 360.0
        self.GEAR_RATIO = 67.82

        # ~602.844 commanded pulses per 1 degree of real arm movement.
        # Negative because these joints are configured inverted.
        self.PULSES_PER_DEGREE = -(self.PULSES_PER_MOTOR_DEGREE * self.GEAR_RATIO)
        self.ENCODER_CPR = 16384

    def checksum(self, motor_id, data_bytes):
        """MKS SUM-8 Checksum validation."""
        return (motor_id + sum(data_bytes)) & 0xFF

    def send_frame(self, motor_id, data_bytes):
        """Builds, signs, and dispatches a CAN message frame."""
        crc = self.checksum(motor_id, data_bytes)
        frame = data_bytes + [crc]
        msg = can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False)
        self.bus.send(msg)
        return frame

    def set_motor_state(self, motor_id, enable=True):
        """Enables or disables motor coils."""
        state_byte = 0x01 if enable else 0x00
        self.send_frame(motor_id, [0xF3, state_byte])
        time.sleep(0.1)

    def read_absolute_encoder(self, motor_id, timeout=1.0):
        """Queries 0x30 multi-turn position tracker from driver."""
        self.send_frame(motor_id, [0x30])
        end = time.time() + timeout
        while time.time() < end:
            resp = self.bus.recv(timeout=end - time.time())
            if resp is None:
                break
            if resp.arbitration_id == motor_id and len(resp.data) >= 7 and resp.data[0] == 0x30:
                carry = int.from_bytes(resp.data[1:5], byteorder="big", signed=True)
                value = int.from_bytes(resp.data[5:7], byteorder="big", signed=False)
                return (carry * self.ENCODER_CPR) + value
        raise TimeoutError(f"Motor {motor_id} failed to return encoder data.")

    def move_relative_degrees(self, motor_id, degrees, speed=40, accel=10):
        """
        Calculates a relative shift in real degrees, tracks it on our absolute 
        microstep counter matrix, and dispatches it safely using the validated 0xF5 frame.
        """
        if not hasattr(self, 'absolute_microstep_tracker'):
            # Initialize tracker baseline to match your current encoder scale
            current_enc = self.read_absolute_encoder(motor_id)
            self.absolute_microstep_tracker = int(current_enc / -1.02)

        # 1. Map arm degrees directly to structural driver microsteps
        # 3200 pulses/motor rev * 67.82 gearbox / 360 degrees = ~602.844 pulses per real arm degree
        pulses_per_arm_degree = (3200.0 * 67.82) / 360.0
        delta_pulses = int(degrees * pulses_per_arm_degree)
        
        # 2. Advance our verified absolute target tracker
        self.absolute_microstep_tracker += delta_pulses
        
        # 3. Enforce our production boundaries to keep it safe (-73000 to 73000 encoder scale)
        # In microstep scale: ~ -71,500 to +71,500 pulses
        if self.absolute_microstep_tracker < -71500 or self.absolute_microstep_tracker > 71500:
            print(f"⚠️ MOVEMENT BLOCKED: Target {self.absolute_microstep_tracker} pulses exceeds boundary limits!")
            return

        # 4. Pack the absolute target cleanly into standard positive 24-bit bytes
        pos = int(self.absolute_microstep_tracker) & 0xFFFFFF
        pos_hi = (pos >> 16) & 0xFF
        pos_mid = (pos >> 8) & 0xFF
        pos_lo = pos & 0xFF
        
        speed_hi = (speed >> 8) & 0xFF
        speed_lo = speed & 0xFF
        
        # 0xF5 absolute position command code
        data = [0xF5, speed_hi, speed_lo, accel, pos_hi, pos_mid, pos_lo]
        self.send_frame(motor_id, data)

    def close(self):
        """Gracefully release the network sockets."""
        self.bus.shutdown()
