#!/usr/bin/env python3
"""
joint_speed_limit_test.py

Measures a joint's usable speed ceiling: SOP Step 9 ("Establish this motor's
speed limit"). Runs a ladder of commanded speeds across a fixed arc and, for
each one, records both

  * actual motor rpm against commanded, and
  * delivered position against commanded position.

Both matter, and the second is the one that bites. Per arctos_motor_guide.tex
the speed field is very close to motor rpm and rolls off gracefully with no
hard stall -- so nothing sounds or looks wrong as the limit is passed. What
actually happens above the limit is that the driver receives more pulses than
it executes and silently loses position. A test that only watched rpm would
report a smooth curve and miss the failure entirely.

Reference figures for the NEMA 17 joints (from the guide, measured no-load):
linear to 225 rpm within 0.5%, 98.7% at 250, rolling off 275-325, saturating
near 310. Joint B is an MKS Servo42D, i.e. NEMA 17 class, so those are the
expected numbers -- but this script exists to measure them for a specific
joint and gearbox rather than assume they transfer.

Method per trial:
  1. Reposition to the start of the arc at --reposition-speed (slow, safe).
  2. Command one 0xFD move across the arc at the trial speed.
  3. Poll the encoder throughout, timestamping every sample.
  4. Steady-state rpm is the median rate over fixed time windows taken from the
     middle of the move, with the acceleration and deceleration ramps trimmed
     off. See steady_rpm() for why the obvious alternatives are wrong.
  5. A whole-move average (arc / time to the FD=2 completion frame) is reported
     alongside it as an independent cross-check. It includes the ramps so it
     always reads low, but it comes from a single interval rather than hundreds,
     so it cannot be distorted by sampling jitter. If the two disagree wildly,
     distrust the steady figure.
  6. Position error is the encoder delta against the commanded arc.

CAUTION. Run with the joint unloaded and mechanically clear -- this drives it
across the arc at up to several hundred rpm. Keep --arc well inside the
joint's safe band. Coils are disabled between trials.

Usage:
    python3 joint_speed_limit_test.py --can-id 0x06 --gear-ratio 67.82 \\
        --arc-lo -20 --arc-hi 20 --band-lo -25 --band-hi 25 \\
        --speeds 25,50,100,150,200,225,250,275,300,325,350
"""
import argparse
import statistics
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


def read_encoder(bus, motor_id, timeout=0.6):
    while bus.recv(timeout=0.0) is not None:
        pass
    send_frame(bus, motor_id, [READ_ENCODER])
    end = time.time() + timeout
    while time.time() < end:
        r = bus.recv(timeout=max(0.0, end - time.time()))
        if r and r.arbitration_id == motor_id and len(r.data) >= 8 and r.data[0] == READ_ENCODER:
            return int.from_bytes(r.data[1:7], "big", signed=True)
    return None


def joint_deg(raw48, gear_ratio):
    return (raw48 * 360.0 / ENCODER_CPR) / gear_ratio


def pulses_for(degrees, gear_ratio):
    return round(abs(degrees) * MICROSTEPS_PER_MOTOR_REV * gear_ratio / 360.0)


def move(bus, motor_id, degrees, gear_ratio, speed, accel):
    direction = 0x80 if degrees < 0 else 0x00
    p = pulses_for(degrees, gear_ratio) & 0xFFFFFF
    data = [POSITION_CONTROL, direction + ((speed >> 8) & 0x0F), speed & 0xFF, accel,
            (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF]
    return send_frame(bus, motor_id, data)


def move_and_sample(bus, motor_id, degrees, gear_ratio, speed, accel, timeout):
    """Run one move, polling the encoder throughout.

    Returns (samples, completed) where samples is [(t, joint_deg), ...]. The
    0xFD status frames share the bus with the encoder replies, so this watches
    for the completion frame while sampling rather than querying for it after.
    """
    while bus.recv(timeout=0.0) is not None:
        pass
    samples = []
    completed = False
    move(bus, motor_id, degrees, gear_ratio, speed, accel)
    t0 = time.time()
    deadline = t0 + timeout
    pending = False
    while time.time() < deadline:
        if not pending:
            send_frame(bus, motor_id, [READ_ENCODER])
            pending = True
        r = bus.recv(timeout=0.05)
        if r is None or r.arbitration_id != motor_id:
            continue
        if len(r.data) >= 8 and r.data[0] == READ_ENCODER:
            raw = int.from_bytes(r.data[1:7], "big", signed=True)
            samples.append((time.time() - t0, joint_deg(raw, gear_ratio)))
            pending = False
        elif len(r.data) == 3 and r.data[0] == POSITION_CONTROL and r.data[1] == 2:
            completed = True
            elapsed = time.time() - t0
            break
    else:
        elapsed = None
    if completed:
        return samples, completed, elapsed
    return samples, completed, (samples[-1][0] if samples else None)


def steady_rpm(samples, gear_ratio, window_s=0.15, ramp_frac=0.25):
    """Steady-state motor rpm, measured over the middle of the move.

    An earlier version took a high percentile of the rate between *consecutive*
    samples. That is unusable: the polling interval is not uniform, so when two
    encoder replies happen to land close together the computed rate spikes, and
    a high percentile selects precisely those spikes. It reported 192 rpm for a
    commanded 150 -- a physically impossible 128% -- which is how the bug was
    caught.

    The fix is to measure each rate over a fixed *time* window rather than one
    sample gap, so jitter averages out, and to take the median rather than a
    high percentile so any remaining outlier cannot dominate. Discarding the
    first and last ramp_frac of the move removes the acceleration and
    deceleration ramps, which is what the percentile was crudely trying to do.
    """
    if len(samples) < 8:
        return None, 0
    t_end = samples[-1][0]
    lo, hi = t_end * ramp_frac, t_end * (1.0 - ramp_frac)
    mid = [(t, d) for t, d in samples if lo <= t <= hi]
    if len(mid) < 4:
        return None, 0
    rates = []
    j = 0
    for i, (ti, di) in enumerate(mid):
        while j < len(mid) and mid[j][0] - ti < window_s:
            j += 1
        if j >= len(mid):
            break
        tj, dj = mid[j]
        dt = tj - ti
        if dt <= 0:
            continue
        rates.append(abs(dj - di) * gear_ratio / 360.0 * 60.0 / dt)
    if len(rates) < 3:
        return None, len(mid)
    return statistics.median(rates), len(mid)


def goto(bus, motor_id, target, gear_ratio, speed, accel, tol=0.15, tries=4):
    """Close the gap to `target` degrees at a safe speed. Encoder is absolute."""
    for _ in range(tries):
        raw = read_encoder(bus, motor_id)
        if raw is None:
            return None
        now = joint_deg(raw, gear_ratio)
        err = target - now
        if abs(err) <= tol:
            return now
        send_frame(bus, motor_id, [ENABLE_MOTOR, 0x01])
        time.sleep(0.2)
        move_and_sample(bus, motor_id, err, gear_ratio, speed, accel,
                        timeout=max(4.0, abs(err) * 1.5 + 3.0))[0]
        time.sleep(0.4)
    raw = read_encoder(bus, motor_id)
    return None if raw is None else joint_deg(raw, gear_ratio)


def main():
    ap = argparse.ArgumentParser(description="Measure a joint's usable speed ceiling")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--gear-ratio", type=float, required=True)
    ap.add_argument("--arc-lo", type=float, required=True)
    ap.add_argument("--arc-hi", type=float, required=True)
    ap.add_argument("--band-lo", type=float, required=True)
    ap.add_argument("--band-hi", type=float, required=True)
    ap.add_argument("--speeds", default="25,50,100,150,200,225,250,275,300,325,350")
    ap.add_argument("--accel", type=int, default=1)
    ap.add_argument("--reposition-speed", type=int, default=25)
    ap.add_argument("--settle-s", type=float, default=0.6)
    ap.add_argument("--pause-s", type=float, default=1.0)
    ap.add_argument("--track-tol", type=float, default=1.0,
                    help="Percent. Within this of commanded counts as tracking.")
    ap.add_argument("--stop-on-loss", action="store_true",
                    help="Halt the ladder as soon as position loss exceeds --track-tol. "
                         "Once the limit is found there is nothing to learn from driving "
                         "faster, and every extra trial is heat into the motor.")
    ap.add_argument("--leave-coils", choices=["on", "off"], default="off")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    speeds = [int(x) for x in args.speeds.split(",") if x.strip()]
    arc = args.arc_hi - args.arc_lo
    if arc <= 0:
        sys.exit("ERROR: --arc-hi must be above --arc-lo.")
    if args.arc_lo < args.band_lo or args.arc_hi > args.band_hi:
        sys.exit(f"ERROR: arc [{args.arc_lo}, {args.arc_hi}] is not inside "
                 f"band [{args.band_lo}, {args.band_hi}].")

    if args.dry_run:
        print("DRY RUN -- no bus opened, nothing sent.")
        print(f"  arc {args.arc_lo} -> {args.arc_hi} = {arc} deg joint "
              f"= {arc * args.gear_ratio / 360.0:.2f} motor revs, {pulses_for(arc, args.gear_ratio)} pulses")
        print(f"  speeds: {speeds}")
        for s in speeds:
            print(f"    speed {s:>4} -> expected ~{arc * args.gear_ratio / 360.0 / (s / 60.0):5.2f}s "
                  f"at {s} rpm")
        print(f"  e-stop -> cansend {args.channel} {args.can_id:03X}#F7"
              f"{checksum(args.can_id, [EMERGENCY_STOP]):02X}")
        return

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    rows = []
    try:
        raw0 = read_encoder(bus, args.can_id)
        if raw0 is None:
            sys.exit("ERROR: no encoder response -- is the driver powered and on the bus?")
        print(f"start: joint_deg={joint_deg(raw0, args.gear_ratio):+.4f}")
        print(f"arc {args.arc_lo:+.1f} -> {args.arc_hi:+.1f} deg "
              f"({arc * args.gear_ratio / 360.0:.2f} motor revs) per trial")
        print(f"e-stop: cansend {args.channel} {args.can_id:03X}#F7"
              f"{checksum(args.can_id, [EMERGENCY_STOP]):02X}\n")
        print(f"{'cmd':>5} {'steady':>8} {'track':>7} {'avg':>7} "
              f"{'moved':>9} {'pos err':>9} {'n':>5} {'steps':>7}")
        print("-" * 68)

        for sp in speeds:
            start = goto(bus, args.can_id, args.arc_lo, args.gear_ratio,
                         args.reposition_speed, args.accel)
            if start is None:
                print(f"{sp:>5}  reposition failed -- stopping.")
                break
            time.sleep(args.settle_s)

            send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x01])
            time.sleep(0.2)
            timeout = max(5.0, arc * args.gear_ratio / 360.0 / (sp / 60.0) * 2.5 + 3.0)
            samples, completed, elapsed = move_and_sample(bus, args.can_id, arc, args.gear_ratio,
                                                          sp, args.accel, timeout)
            time.sleep(args.settle_s)
            raw_end = read_encoder(bus, args.can_id)
            end = joint_deg(raw_end, args.gear_ratio)
            moved = end - start
            rpm, n_mid = steady_rpm(samples, args.gear_ratio)
            track = (rpm / sp * 100.0) if rpm else None
            pos_err = (moved - arc) / arc * 100.0
            # Independent cross-check: whole-move average, ramps included, from
            # one interval instead of hundreds. Always below the steady figure.
            motor_revs = arc * args.gear_ratio / 360.0
            avg = (motor_revs * 60.0 / elapsed) if elapsed else None

            rows.append((sp, rpm, track, moved, pos_err, completed))
            print(f"{sp:>5} {(f'{rpm:8.1f}' if rpm else '     n/a')} "
                  f"{(f'{track:6.1f}%' if track else '    n/a')} "
                  f"{(f'{avg:7.1f}' if avg else '    n/a')} "
                  f"{moved:+9.3f} {pos_err:+8.2f}% {n_mid:>5} "
                  f"{'ok' if completed else 'NO FD2':>7}")

            send_frame(bus, args.can_id, [ENABLE_MOTOR, 0x00])

            if args.stop_on_loss and abs(pos_err) > args.track_tol:
                print(f"\n    position loss of {pos_err:+.2f}% at speed {sp} exceeds "
                      f"{args.track_tol}% -- limit found, stopping the ladder here.")
                break
            time.sleep(args.pause_s)

        # Summary against the SOP's criteria.
        print("\n=== summary ===")
        linear = [r for r in rows if r[2] is not None and abs(100 - r[2]) <= args.track_tol
                  and abs(r[4]) <= args.track_tol]
        if linear:
            print(f"    tracks commanded within {args.track_tol}% up to: {max(r[0] for r in linear)} rpm")
        else:
            print("    no trial tracked within tolerance -- check the arc length and samples")
        rolled = [r for r in rows if r[2] is not None and abs(100 - r[2]) > args.track_tol]
        if rolled:
            print(f"    roll-off begins at: {min(r[0] for r in rolled)} rpm")
        lost = [r for r in rows if abs(r[4]) > args.track_tol]
        if lost:
            print(f"    position loss begins at: {min(r[0] for r in lost)} rpm "
                  f"(this is the number that matters for positioning)")
        else:
            print("    no position loss at any speed tested")
        if rows:
            best = max((r[1] or 0) for r in rows)
            print(f"    highest rpm observed: {best:.1f}")
    finally:
        state = 0x01 if args.leave_coils == "on" else 0x00
        send_frame(bus, args.can_id, [ENABLE_MOTOR, state])
        print(f"    coils left {'ENABLED' if state else 'DISABLED'}")
        bus.shutdown()


if __name__ == "__main__":
    main()
