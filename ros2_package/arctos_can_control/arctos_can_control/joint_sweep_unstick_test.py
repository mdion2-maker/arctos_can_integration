#!/usr/bin/env python3
"""
joint_sweep_unstick_test.py

Timed back-and-forth sweep across a known rough spot in the gear mesh, with
the same back-off-and-retry ("unstick") cycle as joint_unstick_move_test.py.

Motivation (2026-09-02 session, Joint B / 0x06): a newly connected joint
tracked 0.5 deg steps cleanly for ~1 deg, then stalled at ~0.92 deg. With
coils disabled the joint hand-back-drove through that point with gritty
resistance, i.e. a passable rough spot rather than a hard block. The theory
worth testing is that repeatedly working the mesh back and forth through the
spot beds it in. joint_unstick_move_test.py only travels in one direction, so
it can cross such a spot once per run; this script sweeps between two bounds
for a fixed wall-clock duration so the same spot is crossed many times.

Differences from joint_unstick_move_test.py:
  * Sweeps between --sweep-lo and --sweep-hi, reversing at each end, rather
    than taking a fixed number of steps in one direction.
  * Runs for --duration-s of wall clock instead of a step count.
  * A step that stays stuck after --max-unstick cycles reverses the sweep
    instead of aborting the run -- backing away from the spot and coming at
    it again is the whole point here. The run still aborts on the global
    --stall-budget.

CAUTION -- heat. A stalled stepper dumps its full phase current into heat
with nothing moving to carry it away. Nine consecutive full-torque retries
into a hard stop are the suspected cause of an earlier torque loss on this
arm that needed a cooldown to recover. --stall-budget caps the total unstick
cycles for the whole run, and --pause-s holds the duty cycle down. Stop if
the motor becomes hot to the touch; there is no temperature-read opcode on
this driver, so software cannot detect it for you.

CAUTION -- grit. If the rough spot is a hard particle rather than surface
roughness, working back and forth through it can embed it further and wear
the printed teeth. This script is for a spot a hand can already pass through.
Prefer opening and cleaning the gearbox over grinding through it many times.

Never enables ENABLE_SHAFT_PROTECTION (0x88); see the SOP for its
false-positive history.

Usage:
    python3 joint_sweep_unstick_test.py --can-id 0x06 --gear-ratio 67.82 \\
        --sweep-lo -1.5 --sweep-hi 2.5 --step-degrees 0.25 \\
        --backoff-degrees 0.5 --band-lo -25 --band-hi 25 --duration-s 75
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
    """One 0xFD relative move. Direction byte plus an always-positive magnitude."""
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
    ap = argparse.ArgumentParser(description="Timed sweep with back-off/retry unstick cycles")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--gear-ratio", type=float, required=True)
    ap.add_argument("--sweep-lo", type=float, required=True, help="Lower end of the sweep")
    ap.add_argument("--sweep-hi", type=float, required=True, help="Upper end of the sweep")
    ap.add_argument("--step-degrees", type=float, default=0.25)
    ap.add_argument("--backoff-degrees", type=float, default=0.5)
    ap.add_argument("--max-unstick", type=int, default=3,
                    help="Back-off/retry cycles at one spot before reversing the sweep")
    ap.add_argument("--stall-budget", type=int, default=12,
                    help="Total unstick cycles for the whole run before aborting. Heat guard.")
    ap.add_argument("--abort-frac", type=float, default=0.60,
                    help="A step delivering less than this fraction counts as stuck")
    ap.add_argument("--duration-s", type=float, default=75.0)
    ap.add_argument("--band-lo", type=float, required=True, help="Hard safety limit")
    ap.add_argument("--band-hi", type=float, required=True, help="Hard safety limit")
    ap.add_argument("--speed", type=int, default=25)
    ap.add_argument("--accel", type=int, default=1)
    ap.add_argument("--settle-s", type=float, default=0.3)
    ap.add_argument("--pause-s", type=float, default=1.0,
                    help="Pause between steps. Holds the duty cycle down; do not zero it.")
    ap.add_argument("--status-timeout-s", type=float, default=1.5)
    ap.add_argument("--leave-coils", choices=["on", "off"], default="off")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if args.band_lo >= args.band_hi:
        sys.exit(f"ERROR: --band-lo ({args.band_lo}) must be below --band-hi ({args.band_hi}).")
    if args.sweep_lo >= args.sweep_hi:
        sys.exit(f"ERROR: --sweep-lo ({args.sweep_lo}) must be below --sweep-hi ({args.sweep_hi}).")
    # The sweep must sit inside the hard band, not merely overlap it.
    if args.sweep_lo < args.band_lo or args.sweep_hi > args.band_hi:
        sys.exit(f"ERROR: sweep [{args.sweep_lo}, {args.sweep_hi}] is not inside "
                 f"band [{args.band_lo}, {args.band_hi}].")

    if args.dry_run:
        fwd = pulses_for(args.step_degrees, args.gear_ratio)
        back = pulses_for(args.backoff_degrees, args.gear_ratio)
        print("DRY RUN -- no bus opened, nothing sent.")
        print(f"  sweep  [{args.sweep_lo}, {args.sweep_hi}] deg, step {args.step_degrees} deg "
              f"= {fwd} pulses, {abs(args.sweep_hi-args.sweep_lo)/args.step_degrees:.0f} steps per traverse")
        print(f"  unstick back-off {args.backoff_degrees} deg = {back} pulses, "
              f"max {args.max_unstick} per spot, {args.stall_budget} total")
        print(f"  duration {args.duration_s}s, band [{args.band_lo}, {args.band_hi}], "
              f"speed {args.speed}, pause {args.pause_s}s")
        print(f"  e-stop -> cansend {args.channel} {args.can_id:03X}#F7"
              f"{checksum(args.can_id, [EMERGENCY_STOP]):02X}")
        return

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    total_unstick = 0
    traverses = 0
    steps_ok = 0
    stuck_spots = []
    try:
        raw0 = read_encoder(bus, args.can_id)
        if raw0 is None:
            sys.exit("ERROR: no encoder response -- is the driver powered and on the bus?")
        pos = joint_deg(raw0, args.gear_ratio)
        print(f"start: raw48={raw0}  joint_deg={pos:.4f}")
        print(f"sweep [{args.sweep_lo}, {args.sweep_hi}] step={args.step_degrees} "
              f"backoff={args.backoff_degrees} for {args.duration_s}s")
        print(f"e-stop: cansend {args.channel} {args.can_id:03X}#F7"
              f"{checksum(args.can_id, [EMERGENCY_STOP]):02X}\n")

        # Head towards whichever end is further away, so the first traverse is long.
        direction = 1 if (args.sweep_hi - pos) >= (pos - args.sweep_lo) else -1
        deadline = time.time() + args.duration_s

        while time.time() < deadline:
            step = args.step_degrees * direction
            projected = pos + step
            if projected > args.sweep_hi or projected < args.sweep_lo:
                direction *= -1
                traverses += 1
                print(f"--- reached end of sweep, reversing (traverse {traverses}) ---")
                continue

            send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x01])
            time.sleep(0.2)
            while bus.recv(timeout=0.0) is not None:
                pass

            achieved = False
            for cycle in range(args.max_unstick + 1):
                move(bus, args.can_id, step, args.gear_ratio, args.speed, args.accel)
                st = collect_status(bus, args.can_id, args.status_timeout_s)
                time.sleep(args.settle_s)
                now = joint_deg(read_encoder(bus, args.can_id), args.gear_ratio)
                moved = now - pos
                frac = moved / step if step else 0.0

                if frac >= args.abort_frac:
                    tag = "step" if cycle == 0 else f"retry {cycle}"
                    print(f"{tag:<10} {pos:+7.3f} -> {now:+7.3f}  "
                          f"moved {moved:+.4f} of {step:+.4f} ({frac*100:5.1f}%)  FD={st}")
                    pos = now
                    achieved = True
                    steps_ok += 1
                    break

                print(f"{'STUCK':<10} {pos:+7.3f} -> {now:+7.3f}  "
                      f"moved {moved:+.4f} of {step:+.4f} ({frac*100:5.1f}%)  FD={st}")
                if cycle == args.max_unstick:
                    break

                total_unstick += 1
                if total_unstick > args.stall_budget:
                    print(f"\n*** ABORT: stall budget of {args.stall_budget} unstick cycles used. "
                          f"Stopping before this cooks the motor.")
                    send_frame(bus, args.can_id, [EMERGENCY_STOP])
                    deadline = 0
                    break

                # Back off against the direction of travel, then come at it again.
                back = -args.backoff_degrees * direction
                move(bus, args.can_id, back, args.gear_ratio, args.speed, args.accel)
                collect_status(bus, args.can_id, args.status_timeout_s)
                time.sleep(args.settle_s)
                after = joint_deg(read_encoder(bus, args.can_id), args.gear_ratio)
                print(f"{'  unstick':<10} {now:+7.3f} -> {after:+7.3f}  "
                      f"backed {after-now:+.4f} (commanded {back:+.4f})  [{total_unstick}/{args.stall_budget}]")
                pos = after
                time.sleep(args.pause_s)

            if deadline == 0:
                break
            if not achieved:
                # Still stuck after the cycles: retreat and approach from the other
                # side rather than abort. Working it loose is the point of this run.
                stuck_spots.append(round(pos, 3))
                direction *= -1
                print(f"--- still stuck near {pos:+.3f}, reversing to approach from the other side ---")
            time.sleep(args.pause_s)

        final = joint_deg(read_encoder(bus, args.can_id), args.gear_ratio)
        print(f"\n=== final {final:.4f} deg ===")
        print(f"    steps completed : {steps_ok}")
        print(f"    traverses       : {traverses}")
        print(f"    unstick cycles  : {total_unstick} of {args.stall_budget} budget")
        print(f"    stuck spots     : {stuck_spots if stuck_spots else 'none'}")
    finally:
        state = 0x01 if args.leave_coils == "on" else 0x00
        send_frame(bus, args.can_id, [ENABLE_MOTOR, state])
        print(f"    coils left {'ENABLED' if state else 'DISABLED'}")
        bus.shutdown()


if __name__ == "__main__":
    main()
