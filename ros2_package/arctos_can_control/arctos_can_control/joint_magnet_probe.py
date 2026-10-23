#!/usr/bin/env python3
"""
joint_magnet_probe.py

Answers one question: does an external magnet held near the motor's encoder
disturb the reported shaft angle, and by how much?

Read-only. Sends only READ_ENCODER (0x31). Never enables coils, never commands
motion.

Background. The MKS driver's encoder is a magnetic rotary sensor reading a small
magnet glued to the end of the motor shaft. It reports an angle and nothing
else -- there is no "magnet present" output to query. So an external magnet
brought near it is not detected, it is *interference*: it adds to the field the
chip measures and the chip returns a wrong shaft angle.

That makes this a useful test either way. With the shaft held still the reading
should be steady to within a count or two. If it moves when a magnet approaches,
the encoder is being pulled off by that magnet -- which is detectable, but it
also means the magnet is corrupting the position feedback that closed-loop
control depends on.

The script first measures the noise floor at rest, then reports any excursion
beyond it, in encoder counts and in joint degrees.

Usage:
    python3 joint_magnet_probe.py --can-id 0x06 --gear-ratio 67.82
"""
import argparse
import sys
import time

import can

READ_ENCODER = 0x31
ENCODER_CPR = 16384


def checksum(motor_id, data):
    return (motor_id + sum(data)) & 0xFF


def read_raw(bus, motor_id, timeout=0.3):
    while bus.recv(timeout=0.0) is not None:
        pass
    bus.send(can.Message(arbitration_id=motor_id,
                         data=[READ_ENCODER, checksum(motor_id, [READ_ENCODER])],
                         is_extended_id=False))
    end = time.time() + timeout
    while time.time() < end:
        r = bus.recv(timeout=max(0.0, end - time.time()))
        if r and r.arbitration_id == motor_id and len(r.data) >= 8 and r.data[0] == READ_ENCODER:
            return int.from_bytes(r.data[1:7], "big", signed=True)
    return None


def main():
    ap = argparse.ArgumentParser(description="Does a magnet disturb the encoder?")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--can-id", type=lambda x: int(x, 0), required=True)
    ap.add_argument("--gear-ratio", type=float, default=67.82)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--baseline-s", type=float, default=3.0)
    ap.add_argument("--seconds", type=float, default=120.0)
    args = ap.parse_args()

    counts_per_joint_deg = args.gear_ratio * ENCODER_CPR / 360.0
    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    peak = 0
    try:
        # --- noise floor, shaft untouched ---
        print(f"Measuring the noise floor for {args.baseline_s:g}s. "
              f"Keep the magnet AWAY and do not touch the joint.", flush=True)
        samples = []
        end = time.time() + args.baseline_s
        while time.time() < end:
            v = read_raw(bus, args.can_id)
            if v is not None:
                samples.append(v)
            time.sleep(1.0 / args.hz)
        if len(samples) < 5:
            sys.exit("ERROR: not enough encoder responses. Is the joint powered and on the bus?")

        rest = round(sum(samples) / len(samples))
        noise = max(abs(s - rest) for s in samples)
        print(f"  at rest: {rest} counts, noise +/-{noise} counts "
              f"({noise / counts_per_joint_deg:.4f} joint deg)", flush=True)

        # Anything beyond this is more than the sensor's own wobble.
        threshold = max(noise * 4, 8)
        print(f"  will report any excursion over {threshold} counts\n", flush=True)
        print("Now bring the magnet up to the encoder and take it away again, "
              "a few times.", flush=True)
        print("Ctrl+C when done.\n", flush=True)

        end = time.time() + args.seconds
        last_state = False
        while time.time() < end:
            v = read_raw(bus, args.can_id)
            if v is not None:
                dev = v - rest
                peak = max(peak, abs(dev))
                over = abs(dev) > threshold
                if over != last_state:
                    if over:
                        print(f"  MAGNET SEEN   offset {dev:+d} counts "
                              f"({dev / counts_per_joint_deg:+.3f} joint deg)", flush=True)
                    else:
                        print(f"  back to rest", flush=True)
                    last_state = over
            time.sleep(1.0 / args.hz)

    except KeyboardInterrupt:
        print("\n(stopped)", flush=True)
    finally:
        print("\n=== result ===", flush=True)
        print(f"    largest excursion: {peak} counts "
              f"({peak / counts_per_joint_deg:.4f} joint deg)", flush=True)
        if peak > 8:
            print("    The magnet DOES pull the encoder reading.", flush=True)
            print("    Detectable -- but it is corrupting the position feedback,", flush=True)
            print("    so it is not a safe place to mount an endstop magnet.", flush=True)
        else:
            print("    The encoder did not react. The magnet is either too weak,", flush=True)
            print("    too far away, or shielded by the motor's back plate.", flush=True)
        bus.shutdown()


if __name__ == "__main__":
    main()
