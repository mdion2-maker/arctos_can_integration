#!/usr/bin/env python3
"""
joint_position_control_test.py

Tests POSITION_CONTROL (0xFD), the CAN command used by set_zero_position.py
-- the one script in this project that actually performs repeated jogging
moves in a loop as part of the real, documented first-time setup flow
(find_home_position/find_opposite_limit/manual_move_joint, all called from
--init). Neither 0xF4 (RELATIVE_POSITION) nor 0xF5 (ABSOLUTE_POSITION) is
used anywhere in that working flow; 0xFD is. It has never been tried in any
diagnostic script in this document before the 2026-08-27 session, where
repeated 0xF4/0xF5 commands consistently produced a partial "burst then
stop" result regardless of speed/accel.

IMPORTANT -- different pulse scale than every other script in this package:
set_zero_position.py's pulses are MICROSTEP pulses (200 full steps/rev x 16
microsteps/step = 3200 pulses/motor-revolution), NOT the 16384-count/rev
magnetic encoder scale that 0x31 READ_ENCODER and every 0xF4/0xF5 script in
this package use. This script computes the commanded pulses on that 3200
pulses/rev scale (matching set_zero_position.py exactly) while still reading
position back via 0x31 on its native 16384-count encoder scale for
monitoring -- the two scales are not interchangeable.

Frame layout (from set_zero_position.py's relative_motion_by_pulses()):
    [0xFD, direction|((speed>>8)&0b1111), speed&0xFF, accel,
     (pulses>>16)&0xFF, (pulses>>8)&0xFF, pulses&0xFF, checksum]
direction is 0x80 or 0x00, OR'd into the top of the speed-high byte (which
is otherwise only a 4-bit field here, capping speed to 0-4095 in whatever
unit set_zero_position.py's speed parameter represents -- also unconfirmed
against source, since 0xFD is never touched by the C++ driver).

UPDATE 2026-08-27 (later same day): ENABLE_SHAFT_PROTECTION (0x88) -- never
sent anywhere in this project before this session -- was suspected as the
missing piece behind incomplete moves (FD 01 "started" but no FD 02
"complete"), since it is required for READ_SHAFT_PROTECTION_STATE (0x3E) to
detect and report a real stall. Enabling it DID trip 0x3E immediately on the
next command. But a direct comparison -- the IDENTICAL command, same
current, same speed/accel, run again with 0x88 NOT sent -- moved almost
perfectly (99.9% of commanded magnitude) and got both FD 01 and FD 02. That
refutes "genuine stall" as a confirmed conclusion: 0x88 appears capable of
tripping on a false positive, not just a real one, at least in that instance.
Treat any trip as a signal worth investigating, not proof on its own -- it
is OFF BY DEFAULT here (--enable-shaft-protection to turn it on) until this
is better understood. A trip also latches the driver down (see this
document's Section 1) and requires a power-cycle of the main supply to
clear, which is expensive to do by accident on a false positive.

The FD 01/FD 02 status-frame behavior described below remains the primary,
non-latching per-command success signal used by this script.

The driver sends an unsolicited two-frame status sequence per command --
FD 01 ("started") followed, sometimes, by FD 02 ("complete"). Whether FD 02
arrives correlates closely with whether the commanded distance was actually
achieved (confirmed via direct candump-to-encoder correlation). This script
re-sends ENABLE_MOTOR and retries the same step (up to --max-retries times)
whenever FD 02 does not arrive and no stall has been flagged.

Usage:
    python3 joint_position_control_test.py --can-id 0x05 --gear-ratio 67.82 \\
        --home-position-deg 12.873 --opposite-limit-deg -24.230 \\
        --step-degrees 1.0 --num-steps 3 --speed 50 --accel 1 --max-retries 4
"""
import argparse
import sys
import time

import can

READ_ENCODER = 0x31
READ_SHAFT_PROTECTION_STATE = 0x3E
ENABLE_MOTOR = 0xF3
ENABLE_SHAFT_PROTECTION = 0x88
POSITION_CONTROL = 0xFD
EMERGENCY_STOP = 0xF7

ENCODER_CPR = 16384
MICROSTEPS_PER_MOTOR_REV = 3200.0  # 200 full steps/rev x 16 microsteps/step, per set_zero_position.py


def checksum(motor_id, data):
    return (motor_id + sum(data)) & 0xFF


def send_frame(bus, motor_id, data):
    crc = checksum(motor_id, data)
    frame = data + [crc]
    bus.send(can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False))
    return frame


def query(bus, motor_id, opcode, expect_len, timeout=0.5):
    while bus.recv(timeout=0.0) is not None:
        pass
    send_frame(bus, motor_id, [opcode])
    end = time.time() + timeout
    while time.time() < end:
        resp = bus.recv(timeout=max(0.0, end - time.time()))
        if resp is None:
            continue
        if resp.arbitration_id == motor_id and len(resp.data) >= expect_len and resp.data[0] == opcode:
            return resp.data
    return None


def read_encoder(bus, motor_id):
    d = query(bus, motor_id, READ_ENCODER, 8)
    if d is None:
        return None
    return int.from_bytes(d[1:7], byteorder="big", signed=True)


def joint_deg(raw48, gear_ratio):
    return (raw48 * 360.0 / ENCODER_CPR) / gear_ratio


def move_position_control(bus, motor_id, pulses, speed, accel, direction):
    pulses = int(pulses) & 0xFFFFFF
    data = [
        POSITION_CONTROL,
        direction + ((speed >> 8) & 0b1111),
        speed & 0xFF,
        accel,
        (pulses >> 16) & 0xFF,
        (pulses >> 8) & 0xFF,
        pulses & 0xFF,
    ]
    return send_frame(bus, motor_id, data)


def emergency_stop(bus, motor_id):
    send_frame(bus, motor_id, [EMERGENCY_STOP])


def watch_fd_status_frames(bus, motor_id, timeout):
    """Collect every unsolicited POSITION_CONTROL status frame (3 bytes:
    FD <status> <crc>) seen in the window. On 2026-08-27, status=1 ("started")
    was sent for essentially every command, but status=2 ("complete") only
    arrived when the commanded distance was actually achieved -- confirmed by
    direct candump-to-encoder correlation. Absence of status=2 is the
    reliable signal that a command needs to be retried."""
    seen = []
    end = time.time() + timeout
    while time.time() < end:
        resp = bus.recv(timeout=max(0.0, end - time.time()))
        if resp is None:
            continue
        if resp.arbitration_id == motor_id and len(resp.data) == 3 and resp.data[0] == POSITION_CONTROL:
            seen.append(resp.data[1])
    return seen


def main():
    parser = argparse.ArgumentParser(description="POSITION_CONTROL (0xFD) jog test")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    parser.add_argument("--gear-ratio", type=float, required=True)
    parser.add_argument("--home-position-deg", type=float, required=True)
    parser.add_argument("--opposite-limit-deg", type=float, required=True)
    parser.add_argument("--safety-margin-deg", type=float, default=2.0)
    parser.add_argument("--step-degrees", type=float, default=1.0,
                         help="Magnitude of joint-degrees per jog step (sign sets --direction default)")
    parser.add_argument("--direction", type=lambda x: int(x, 0), default=None,
                         help="0x80 or 0x00; defaults based on the sign of --step-degrees")
    parser.add_argument("--num-steps", type=int, default=3)
    parser.add_argument("--speed", type=int, default=50)
    parser.add_argument("--accel", type=int, default=1)
    parser.add_argument("--inter-step-pause-s", type=float, default=1.0)
    parser.add_argument("--max-retries", type=int, default=4,
                         help="Retries per step if no FD status=2 'complete' frame arrives")
    parser.add_argument("--status-timeout-s", type=float, default=3.0,
                         help="How long to wait for status frames after each attempt")
    parser.add_argument("--enable-shaft-protection", action="store_true",
                         help="Send ENABLE_SHAFT_PROTECTION (0x88) at startup. CAUTION: on "
                              "2026-08-27 this caused instant trips on commands that moved "
                              "perfectly when protection was NOT enabled -- treat trips as a "
                              "possible false positive, not confirmed proof of a real stall, "
                              "until cross-checked against an unprotected run of the same command. "
                              "Off by default for that reason.")
    args = parser.parse_args()

    lo = args.opposite_limit_deg + args.safety_margin_deg
    hi = args.home_position_deg - args.safety_margin_deg

    def within_safe_band(deg):
        return lo <= deg <= hi

    direction = args.direction
    if direction is None:
        direction = 0x80 if args.step_degrees < 0 else 0x00

    pulses_per_joint_degree = MICROSTEPS_PER_MOTOR_REV * args.gear_ratio / 360.0
    step_pulses = round(abs(args.step_degrees) * pulses_per_joint_degree)

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    try:
        raw0 = read_encoder(bus, args.can_id)
        deg0 = joint_deg(raw0, args.gear_ratio)
        print(f"Start: raw48={raw0}  joint_deg={deg0:.4f}")

        if args.enable_shaft_protection:
            print("Sending ENABLE_SHAFT_PROTECTION (0x88, 0x01) -- CAUTION: this has caused "
                  "instant false-positive trips on commands that otherwise moved correctly. "
                  "See the SOP's session log before trusting a trip as a real stall.")
            send_frame(bus, args.can_id, [ENABLE_SHAFT_PROTECTION, 0x01])
            time.sleep(0.3)

        print(f"Safe band: [{lo:.3f}, {hi:.3f}] deg")
        print(f"step_pulses={step_pulses} (microstep scale, direction=0x{direction:02X}), "
              f"num_steps={args.num_steps}, speed={args.speed}, accel={args.accel}\n")

        worst_case_deg = deg0 + (args.step_degrees * args.num_steps)
        if not within_safe_band(worst_case_deg):
            print(f"ABORT: worst-case final position {worst_case_deg:.3f} deg would leave the safe band.")
            return

        expected = -abs(args.step_degrees) if direction == 0x80 else abs(args.step_degrees)
        prev_deg = deg0
        steps_confirmed = 0
        for i in range(1, args.num_steps + 1):
            confirmed = False
            for attempt in range(1, args.max_retries + 2):  # 1 initial try + max_retries retries
                send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x01])
                time.sleep(0.3)
                while bus.recv(timeout=0.0) is not None:
                    pass

                frame = move_position_control(bus, args.can_id, step_pulses, args.speed, args.accel, direction)
                tag = f"[Step {i}, attempt {attempt}]"
                print(f"{tag} Sent POSITION_CONTROL pulses={step_pulses} dir=0x{direction:02X}: "
                      f"{[hex(b) for b in frame]}")

                statuses = watch_fd_status_frames(bus, args.can_id, args.status_timeout_s)
                print(f"{tag} FD statuses seen: {statuses}")

                prot = query(bus, args.can_id, READ_SHAFT_PROTECTION_STATE, 3)
                if prot is not None and len(prot) >= 2 and prot[1] not in (0,):
                    print(f"{tag} *** Shaft protection TRIPPED (status={prot[1]}) -- a genuine stall. ***")
                    print("The driver is now latched down. Power-cycle its main supply to clear it "
                          "before running anything else -- retrying will not help.")
                    emergency_stop(bus, args.can_id)
                    sys.exit(1)

                raw_now = read_encoder(bus, args.can_id)
                deg_now = joint_deg(raw_now, args.gear_ratio) if raw_now is not None else None
                if deg_now is not None and not within_safe_band(deg_now):
                    print(f"{tag} Left the safe band -- emergency stopping.")
                    emergency_stop(bus, args.can_id)
                    sys.exit(1)

                if 2 in statuses:
                    confirmed = True
                    break
                print(f"{tag} No status=2 'complete' frame -- retrying this step.")

            current_raw = read_encoder(bus, args.can_id)
            current_deg = joint_deg(current_raw, args.gear_ratio)
            moved = current_deg - prev_deg
            outcome = "CONFIRMED" if confirmed else f"NOT CONFIRMED after {args.max_retries + 1} attempts"
            print(f"[Step {i}] result: now at {current_deg:.4f} deg "
                  f"(moved {moved:+.4f}, expected approx {expected:+.4f}) -- {outcome}\n")
            if confirmed:
                steps_confirmed += 1
            prev_deg = current_deg
            time.sleep(args.inter_step_pause_s)

        final_raw = read_encoder(bus, args.can_id)
        final_deg = joint_deg(final_raw, args.gear_ratio)
        net_change = final_deg - deg0
        total_commanded = args.step_degrees * args.num_steps
        print(f"=== Final: raw48={final_raw}  joint_deg={final_deg:.4f}  "
              f"(started {deg0:.4f}, net change {net_change:+.4f}) ===")
        print(f"{steps_confirmed}/{args.num_steps} steps confirmed via FD status=2. "
              f"Net change {net_change:+.4f} deg vs {total_commanded:+.4f} deg total commanded.")

    finally:
        print("\nDisabling motor coils.")
        send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x00])
        bus.shutdown()


if __name__ == "__main__":
    main()
