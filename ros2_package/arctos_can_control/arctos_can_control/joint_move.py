#!/usr/bin/env python3
"""
joint_move.py

Move one joint a given number of degrees, in a named direction, safely.

    python3 joint_move.py z cw 15
    python3 joint_move.py x ccw 30
    python3 joint_move.py y cw 40 --speed 50

Three things to change: the joint, the direction, the distance. Everything
else -- CAN ID, gear ratio, which direction byte means clockwise, which IO bits
are real on that board -- is looked up from the table below, because every one of
those differs per joint and getting one wrong has cost this project real time.

Why a direction *name* rather than a byte. The direction byte does not transfer
between joints: 0x00 is clockwise on X and Z, counter-clockwise on Y and C. Each
mapping below was established by eye, on the machine. A joint whose mapping has
never been confirmed is marked None and this script refuses to move it rather
than guess, because a guess is a 50/50 chance of driving the wrong way.

Why the hardware type matters. On the 42D only IN_1 (bit 0) exists; bit 1 reads 0
permanently because there is no IN_2 to report. Treating that as a triggered
limit makes every 42D look like it is sitting on an endstop. The mask below keeps
each board's phantom bits out of the abort logic.

Safety, all of it on by default:
  * Endstop abort. Any watched input going low stops the move immediately.
    A limit already low at the start is ignored -- so you can back off one --
    but it re-arms the moment it reads high again.
  * Stall abort. A hard stop, a bind, or gears slipping to a halt moves no
    endstop bit and is otherwise invisible. If travel falls below 5% of the
    expected rate the move is stopped.
  * EMERGENCY_STOP before the coils are disabled. Disabling coils does NOT
    cancel an in-flight move -- measured on Joint X, a 5 deg move whose script
    exited at 0.5 deg completed the remaining 4.5 deg with nothing running.
    Only 0xF7 actually stops it.

What this script cannot tell you: whether the ARM moved. The encoder is on the
motor shaft, upstream of the reduction, so slipping gears produce a flawless
trace for an output going nowhere. Watch the joint, not the numbers.
"""
import argparse
import collections
import sys
import time

import can

ENCODER_CPR = 16384
MICROSTEPS_PER_REV = 3200.0
ACCEL = 2
STALL_WINDOW_S = 1.5
STALL_GRACE_S = 4.0
STALL_FRACTION = 0.05          # of expected travel per window

# can_id, gear ratio, board, clockwise byte, limit-port-remap enabled
# The clockwise byte is None where it has never been confirmed on the machine.
JOINTS = {
    "x": (0x01, 13.5,  "57D", 0x00, False),
    "y": (0x02, 150.0, "57D", 0x80, False),
    "z": (0x03, 150.0, "42D", 0x00, True),   # remapped 2026-09-11
    "a": (0x04, 48.0,  "42D", None, False),
    "b": (0x05, 67.82, "42D", None, False),
    "c": (0x06, 67.82, "42D", 0x80, True),   # remapped 2026-09-03
}


def io_mask(board, remapped):
    """Which IO bits are real inputs, so phantom bits stay out of the aborts.

    A 57D has IN_1 and IN_2 both. A bare 42D has only IN_1, and bit 1 reads 0
    forever because there is no IN_2 to report -- reading that as a triggered
    limit makes every 42D look parked on an endstop. But once limit port remap
    is on, a 42D gains a second real input: IN_1 reports En and IN_2 reports
    Dir. So the mask follows the remap state, not the board type. Getting this
    wrong on a remapped 42D whose bit 0 happens to be low arms nothing at all
    and moves the joint with no endstop protection.
    """
    if board == "57D" or remapped:
        return 0b11
    return 0b01


def checksum(motor_id, data):
    return (motor_id + sum(data)) & 0xFF


def send(bus, motor_id, data):
    bus.send(can.Message(arbitration_id=motor_id,
                         data=data + [checksum(motor_id, data)],
                         is_extended_id=False))


def query(bus, motor_id, opcode, timeout=0.25):
    while bus.recv(timeout=0.0) is not None:
        pass
    send(bus, motor_id, [opcode])
    end = time.time() + timeout
    while time.time() < end:
        r = bus.recv(timeout=max(0.0, end - time.time()))
        if r is not None and r.arbitration_id == motor_id and r.data[0] == opcode:
            if len(r.data) == 3:
                return r.data[1]
            if len(r.data) == 8:
                return int.from_bytes(r.data[1:7], "big", signed=True)
    return None


def main():
    ap = argparse.ArgumentParser(
        description="Move one Arctos joint a given distance, with endstop and stall aborts.")
    ap.add_argument("joint", choices=sorted(JOINTS), help="which joint")
    ap.add_argument("direction", choices=["cw", "ccw"], help="clockwise or counter-clockwise")
    ap.add_argument("degrees", type=float, help="joint degrees to travel")
    ap.add_argument("--speed", type=int, default=25,
                    help="motor rpm (default 25). Joint Y's gears slip above 25.")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--dry-run", action="store_true", help="print the frame, send nothing")
    args = ap.parse_args()

    can_id, gear, board, cw_byte, remapped = JOINTS[args.joint]
    if cw_byte is None:
        sys.exit(f"Joint {args.joint.upper()} has no confirmed direction mapping.\n"
                 f"Move it once with a known byte, watch which way it turns, and put the\n"
                 f"clockwise byte in the JOINTS table. Refusing to guess.")
    direction = cw_byte if args.direction == "cw" else (cw_byte ^ 0x80)

    if args.degrees <= 0:
        sys.exit("degrees must be positive -- use the direction argument to reverse")

    mask = io_mask(board, remapped)
    counts_per_joint_deg = gear * ENCODER_CPR / 360.0
    motor_rev = args.degrees * gear / 360.0
    pulses = round(motor_rev * MICROSTEPS_PER_REV)
    expected_s = motor_rev / args.speed * 60.0
    stall_min = max(200, int(args.speed / 60.0 * ENCODER_CPR * STALL_WINDOW_S * STALL_FRACTION))

    frame = [0xFD, direction + ((args.speed >> 8) & 0x0F), args.speed & 0xFF, ACCEL,
             (pulses >> 16) & 0xFF, (pulses >> 8) & 0xFF, pulses & 0xFF]

    print(f"joint {args.joint.upper()}  CAN 0x{can_id:02X}  {board}  {gear}:1"
          f"{'  (limit port remap ON)' if remapped else ''}")
    print(f"{args.degrees} deg {args.direction} = {motor_rev:.3f} motor rev = {pulses} pulses "
          f"at speed {args.speed}  (~{expected_s:.0f}s)")
    print(f"direction byte 0x{direction:02X}   watching IO bits "
          f"{[b for b in (0, 1) if (mask >> b) & 1]}")

    if args.dry_run:
        print("DRY RUN -- nothing sent.")
        print(f"  cansend {args.channel} {can_id:03X}#"
              + "".join(f"{b:02X}" for b in frame + [checksum(can_id, frame)]))
        return

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    stopped_by = "bound reached"
    try:
        p0 = query(bus, can_id, 0x31)
        io0 = query(bus, can_id, 0x34)
        if p0 is None or io0 is None:
            sys.exit(f"No reply from 0x{can_id:02X}. Powered? On the bus? Run can_id_scan.py.")
        armed = io0 & mask
        print(f"start raw={p0}  io=0x{io0:02X}")
        if armed != mask:
            print(f"  note: input(s) {[b for b in (0,1) if (mask>>b)&1 and not (armed>>b)&1]} "
                  f"already low -- ignored until they read high again")

        send(bus, can_id, [0xF3, 0x01])
        time.sleep(0.3)
        send(bus, can_id, frame)

        hist = collections.deque()
        t0 = last_print = time.time()
        while time.time() - t0 < expected_s + 30:
            io = query(bus, can_id, 0x34)
            pos = query(bus, can_id, 0x31)
            now = time.time()
            if io is None or pos is None:
                continue
            travelled = (pos - p0) / counts_per_joint_deg

            armed |= (io & mask)                       # re-arm once an input reads high
            newly_low = armed & ~io & mask
            if newly_low:
                send(bus, can_id, [0xF7])
                bit = 0 if newly_low & 1 else 1
                print(f"\n*** ENDSTOP IN_{bit + 1} at {now - t0:.1f}s "
                      f"-- io=0x{io:02X}, travelled {travelled:+.3f} deg, raw={pos}")
                stopped_by = f"endstop IN_{bit + 1}"
                break

            hist.append((now, pos))
            while hist and now - hist[0][0] > STALL_WINDOW_S:
                hist.popleft()
            if (now - t0 > STALL_GRACE_S and len(hist) > 5
                    and abs(pos - hist[0][1]) < stall_min):
                send(bus, can_id, [0xF7])
                print(f"\n*** STALL at {now - t0:.1f}s -- {abs(pos - hist[0][1])} counts in "
                      f"{now - hist[0][0]:.1f}s. Hard stop, bind, or slipping to a halt.")
                print(f"    travelled {travelled:+.3f} deg, raw={pos}")
                stopped_by = "stall"
                break

            if now - last_print > 5.0:
                print(f"  t={now - t0:5.1f}s  {travelled:+8.3f} deg  io=0x{io:02X}", flush=True)
                last_print = now
            if abs(pos - p0) >= pulses * ENCODER_CPR / MICROSTEPS_PER_REV - 300:
                print(f"  arrived: {travelled:+.3f} deg")
                break
    finally:
        send(bus, can_id, [0xF7])       # a coil disable does NOT cancel a move
        time.sleep(0.1)
        send(bus, can_id, [0xF3, 0x00])
        # The driver may have stopped answering -- lost power, tripped supply,
        # pulled connector. The stop and the coil disable above have already gone
        # out; this is only reporting, so it must never raise and prevent the
        # bus being shut down cleanly.
        end_pos = query(bus, can_id, 0x31)
        end_io = query(bus, can_id, 0x34)
        pos_s = str(end_pos) if end_pos is not None else "no reply"
        io_s = f"0x{end_io:02X}" if end_io is not None else "no reply"
        print(f"\nfinal raw={pos_s}  io={io_s}  ({stopped_by}).  coils disabled")
        if end_pos is None or end_io is None:
            print("  WARNING: the driver stopped answering. Zero bus-errors with total\n"
                  "  silence means it lost power rather than the bus failing -- check the\n"
                  "  supply, and whether it tripped because the joint met a hard stop.")
        bus.shutdown()


if __name__ == "__main__":
    main()
