#!/usr/bin/env python3
"""
joint_unstick_move_test.py

Stepped 0xFD POSITION_CONTROL move with an automatic back-off-and-retry
("unstick") cycle for joints with a gritty or tight spot in the gear mesh.

Motivation (2026-09-01 session, Joint C): after a newly printed gearbox was
fitted, the joint tracked 2.0 deg steps to within 0.008 deg for 12 deg of
travel, then began stalling. Hand back-driving with the coils disabled found
gritty resistance in both directions that a hand could always push through
easily -- i.e. a passable rough spot, not a chipped tooth or a hard block.
A plain retry (as in joint_position_control_test.py --max-retries) just
re-commands the SAME move into the stuck position, which does nothing except
dump stall current into the motor as heat. This script instead backs OFF by
--backoff-degrees, then re-attempts the original step, giving the mesh a
chance to re-seat before trying again.

CAUTION -- heat. A stalled stepper converts its full phase current into heat
with no motion to carry it away. Nine consecutive full-torque retries into a
hard stop earlier in that same session are the suspected cause of a torque
loss that took a cooldown to recover from. --pause-s (default 1.5s) exists
to hold the duty cycle down; do not set it to 0 for a long run, and stop if
the motor becomes hot to the touch. There is no known temperature-read
opcode on this driver, so the software cannot detect this for you.

CAUTION -- grit. If the rough spot is a hard particle rather than surface
roughness, working back and forth through it repeatedly can embed it further
and accelerate wear on printed teeth. Prefer opening and cleaning the
gearbox over grinding through it many times.

Never enables ENABLE_SHAFT_PROTECTION (0x88); see the SOP for its
false-positive history. Leaves coils in the state selected by --leave-coils.

Usage:
    python3 joint_unstick_move_test.py --can-id 0x05 --gear-ratio 67.82 \\
        --step-degrees 2.0 --num-steps 10 --backoff-degrees 0.3 \\
        --max-unstick 5 --band-lo -25 --band-hi 25
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
MICROSTEPS_PER_MOTOR_REV = 3200.0


def checksum(motor_id, data):
    return (motor_id + sum(data)) & 0xFF


def build_frame(motor_id, data):
    return data + [checksum(motor_id, data)]


def send_frame(bus, motor_id, data):
    frame = build_frame(motor_id, data)
    if bus is not None:
        bus.send(can.Message(arbitration_id=motor_id, data=frame, is_extended_id=False))
    return frame


def query(bus, motor_id, opcode, expect_len, timeout=0.6):
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
    return None if d is None else int.from_bytes(d[1:7], "big", signed=True)


def joint_deg(raw48, gear_ratio):
    return (raw48 * 360.0 / ENCODER_CPR) / gear_ratio


def pulses_for(degrees, gear_ratio):
    return round(abs(degrees) * MICROSTEPS_PER_MOTOR_REV * gear_ratio / 360.0)


def move(bus, motor_id, degrees, gear_ratio, speed, accel):
    """Send one 0xFD relative move. Direction byte + always-positive magnitude."""
    direction = 0x80 if degrees < 0 else 0x00
    p = pulses_for(degrees, gear_ratio) & 0xFFFFFF
    data = [POSITION_CONTROL, direction + ((speed >> 8) & 0x0F), speed & 0xFF, accel,
            (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF]
    return send_frame(bus, motor_id, data)


def collect_status(bus, motor_id, timeout):
    seen = []
    end = time.time() + timeout
    while time.time() < end:
        r = bus.recv(timeout=max(0.0, end - time.time()))
        if r and r.arbitration_id == motor_id and len(r.data) == 3 and r.data[0] == POSITION_CONTROL:
            seen.append(r.data[1])
    return seen


def main():
    ap = argparse.ArgumentParser(description="Stepped move with back-off/retry unstick cycle")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--gear-ratio", type=float, required=True)
    ap.add_argument("--step-degrees", type=float, default=2.0)
    ap.add_argument("--num-steps", type=int, default=10)
    ap.add_argument("--backoff-degrees", type=float, default=0.3,
                    help="How far to reverse before re-attempting a stuck step")
    ap.add_argument("--max-unstick", type=int, default=5,
                    help="Back-off/retry cycles per step before aborting")
    ap.add_argument("--abort-frac", type=float, default=0.60,
                    help="A step delivering less than this fraction counts as stuck")
    ap.add_argument("--band-lo", type=float, required=True)
    ap.add_argument("--band-hi", type=float, required=True)
    ap.add_argument("--speed", type=int, default=25)
    ap.add_argument("--accel", type=int, default=1)
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--pause-s", type=float, default=1.5,
                    help="Pause between attempts. Holds the duty cycle down; do not zero it.")
    ap.add_argument("--status-timeout-s", type=float, default=3.0)
    ap.add_argument("--leave-coils", choices=["on", "off"], default="on")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print the frames that would be sent and exit. No bus, no motion.")
    args = ap.parse_args()

    if args.band_lo >= args.band_hi:
        sys.exit(f"ERROR: --band-lo ({args.band_lo}) must be below --band-hi ({args.band_hi}).")

    if args.dry_run:
        step_f = build_frame(args.can_id, [POSITION_CONTROL,
                    (0x80 if args.step_degrees < 0 else 0x00) + ((args.speed >> 8) & 0x0F),
                    args.speed & 0xFF, args.accel,
                    (pulses_for(args.step_degrees, args.gear_ratio) >> 16) & 0xFF,
                    (pulses_for(args.step_degrees, args.gear_ratio) >> 8) & 0xFF,
                    pulses_for(args.step_degrees, args.gear_ratio) & 0xFF])
        back_deg = -args.backoff_degrees if args.step_degrees > 0 else args.backoff_degrees
        back_f = build_frame(args.can_id, [POSITION_CONTROL,
                    (0x80 if back_deg < 0 else 0x00) + ((args.speed >> 8) & 0x0F),
                    args.speed & 0xFF, args.accel,
                    (pulses_for(back_deg, args.gear_ratio) >> 16) & 0xFF,
                    (pulses_for(back_deg, args.gear_ratio) >> 8) & 0xFF,
                    pulses_for(back_deg, args.gear_ratio) & 0xFF])
        print("DRY RUN -- no bus opened, nothing sent.")
        print(f"  step   {args.step_degrees:+.3f} deg = {pulses_for(args.step_degrees, args.gear_ratio)} pulses"
              f"  -> {[hex(b) for b in step_f]}")
        print(f"  backoff{back_deg:+.3f} deg = {pulses_for(back_deg, args.gear_ratio)} pulses"
              f"  -> {[hex(b) for b in back_f]}")
        print(f"  e-stop -> {[hex(b) for b in build_frame(args.can_id, [EMERGENCY_STOP])]}"
              f"   (cansend {args.channel} {args.can_id:03X}#F7{checksum(args.can_id,[EMERGENCY_STOP]):02X})")
        print(f"  band [{args.band_lo}, {args.band_hi}]  max_unstick={args.max_unstick}")
        return

    back_deg = -args.backoff_degrees if args.step_degrees > 0 else args.backoff_degrees
    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    total_unstick = 0
    try:
        raw0 = read_encoder(bus, args.can_id)
        if raw0 is None:
            sys.exit("ERROR: no encoder response -- is the driver powered and on the bus?")
        deg0 = joint_deg(raw0, args.gear_ratio)
        print(f"start: raw48={raw0}  joint_deg={deg0:.4f}")
        print(f"step={args.step_degrees:+.3f}deg  backoff={back_deg:+.3f}deg  "
              f"max_unstick={args.max_unstick}  band=[{args.band_lo}, {args.band_hi}]")
        print(f"e-stop: cansend {args.channel} {args.can_id:03X}#F7"
              f"{checksum(args.can_id,[EMERGENCY_STOP]):02X}\n")

        prev = deg0
        for i in range(1, args.num_steps + 1):
            projected = prev + args.step_degrees
            if not (args.band_lo <= projected <= args.band_hi):
                print(f"[{i}] projected {projected:.4f} leaves band -- stopping cleanly.")
                break

            achieved = False
            for cycle in range(args.max_unstick + 1):
                send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x01])
                time.sleep(0.3)
                while bus.recv(timeout=0.0) is not None:
                    pass

                move(bus, args.can_id, args.step_degrees, args.gear_ratio, args.speed, args.accel)
                st = collect_status(bus, args.can_id, args.status_timeout_s)
                time.sleep(args.settle_s)
                now = joint_deg(read_encoder(bus, args.can_id), args.gear_ratio)
                moved = now - prev
                frac = moved / args.step_degrees if args.step_degrees else 0.0
                label = "step" if cycle == 0 else f"retry {cycle}/{args.max_unstick}"
                print(f"[{i}] {label:<16} moved {moved:+.4f} of {args.step_degrees:+.4f} "
                      f"({frac*100:6.1f}%)  FD={st}  at {now:.4f}deg")

                if frac >= args.abort_frac:
                    achieved = True
                    prev = now
                    break

                if cycle == args.max_unstick:
                    break

                # Unstick: back off, let the mesh re-seat, then re-attempt.
                total_unstick += 1
                move(bus, args.can_id, back_deg, args.gear_ratio, args.speed, args.accel)
                collect_status(bus, args.can_id, args.status_timeout_s)
                time.sleep(args.settle_s)
                after_back = joint_deg(read_encoder(bus, args.can_id), args.gear_ratio)
                print(f"[{i}] {'unstick '+str(cycle+1):<16} backed {after_back-now:+.4f} "
                      f"(commanded {back_deg:+.4f})  at {after_back:.4f}deg")
                prev = after_back
                time.sleep(args.pause_s)

            if not achieved:
                print(f"\n*** ABORT: step {i} still stuck after {args.max_unstick} "
                      f"back-off/retry cycles. Sending EMERGENCY_STOP.")
                send_frame(bus, args.can_id, [EMERGENCY_STOP])
                break
            time.sleep(args.pause_s)

        final_raw = read_encoder(bus, args.can_id)
        final = joint_deg(final_raw, args.gear_ratio)
        print(f"\n=== final {final:.4f}deg (net {final-deg0:+.4f} from {deg0:.4f}) ===")
        print(f"    unstick cycles used: {total_unstick}")
    finally:
        state = 0x01 if args.leave_coils == "on" else 0x00
        send_frame(bus, args.can_id, [ENABLE_MOTOR, state])
        print(f"    coils left {'ENABLED' if state else 'DISABLED'}")
        bus.shutdown()


if __name__ == "__main__":
    main()
