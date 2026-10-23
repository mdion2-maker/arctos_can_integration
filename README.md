# Arctos Arm — Bring-Up and Control

Tooling, firmware and a working record for bringing an
[Arctos](https://github.com/Arctos-Robotics/ros2_arctos) 6-axis robotic arm from a box
of parts to a machine that moves accurately under command.

> **Status: in progress, and honest about it.** All six joints and the gripper are on
> the bus and individually verified. The arm has not yet been driven as a whole. Some
> findings recorded here were later proved wrong and are kept with corrections
> attached, because knowing which dead ends were already walked is most of the value.

This started as a CAN bus investigation and outgrew it. It now covers joint bring-up,
endstop wiring and verification, homing, travel calibration, gear-slip measurement,
and a CAN-connected gripper running its own firmware. It does **not** vendor the
upstream `ros2_arctos` repository — everything here is built on top of it.

---

## Where the project is now

| Joint | Driver | Gear | Endstops | Verified | Notes |
|---|---|---|---|---|---|
| **X** | 57D | 13.5:1 | one, on `IN_2` | homed, zeroed | **Never send `GoHome`** — `IN_1` is empty, so it would never stop |
| **Y** | 57D | 150:1 | two | homed, 158.24° envelope | Gears slip above speed 25 |
| **Z** | 42D | 150:1 | two, via remap | 124.4° ± 0.3 envelope | Needs limit port remap |
| **A** | 42D | 48:1 | **damaged** | bare motor only | Endstop killed by reversed supply; never coupled to its joint |
| **B** | 42D | 67.82:1 | appears remapped | on the bus | **Direction unconfirmed** — tooling refuses to move it |
| **C** | 42D | 67.82:1 | two, via remap | on the bus | Differential wrist with B |
| **Gripper** | Arduino + MCP2515 | — | — | validated | CAN ID `0x07`, band 100–118° |

**Next up:** a staged motion test across X, Y, Z and A on the fully assembled arm, then
B and C together as a differential wrist.

---

## Start here

**[`docs/Engineering_Log.pdf`](docs/Engineering_Log.pdf)** is the primary document — a
dated, chronological record of every session, finding, dead end and correction.
Procedural material lives in its appendices. Read the entry for whatever joint you are
touching before you touch it.

The other four documents are standing reference rather than history:

| Document | What it is for |
|---|---|
| [`Arctos_CAN_Interface`](docs/Arctos_CAN_Interface.pdf) | How the bus and MKS protocol work: command reference, per-joint parameters, endstops, homing, troubleshooting. **Read this first.** |
| [`BASIC_CAN_COMMANDS`](docs/BASIC_CAN_COMMANDS.pdf) | Ready-made terminal commands — bring the bus up, query, zero, shut down. |
| [`Nema_17_MKS42D_SOP`](docs/Nema_17_MKS42D_SOP.pdf) | Step-by-step bring-up of a 42D joint (Z, A, B, C). |
| [`Nema_23_MKS57D_SOP`](docs/Nema_23_MKS57D_SOP.pdf) | Step-by-step bring-up of a 57D joint (X, Y). |

Each ships as `.tex` and `.pdf`. Code blocks carry no line numbers, so anything in them
pastes straight into a terminal.

---

## Step by step

### 1. Bring the CAN interface up

```bash
sudo pkill slcand
sudo ip link delete can0 2>/dev/null
sudo slcand -o -c -s6 /dev/serial/by-id/usb-Openlight_Labs_CANable2* can0
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
ip -br link show can0          # expect: can0  UP
```

This must be redone every time the adapter is replugged or re-enumerates — `slcand`
does not restart itself.

### 2. Check the bus is quiet, then find what is on it

```bash
candump can0                   # with nothing transmitting, expect ZERO frames
python3 ros2_package/arctos_can_control/arctos_can_control/can_id_scan.py
```

**A flood of frames here means a termination fault, not a broken driver.** The bus
needs exactly two 120 Ω terminators, one at each physical end. Three makes a node
answer its own reflections at ~1,500 frames/second and takes the whole bus down. With
power off, `CAN_H` to `CAN_L` should read **60 Ω** — 40 Ω means one too many.

Never trust a config file or a DIP switch for a CAN ID. Every session in this project
found at least one mismatch between what was assumed and what actually answered.

### 3. Check a joint's health before moving it

```bash
python3 .../joint_reliability_test.py --can-id 0x01 --gear-ratio 13.5
python3 .../joint_endstop_test.py --can-id 0x01
```

Both are read-only. The second watches the IO byte while you pass a magnet by hand —
do this before relying on any endstop.

**On a 42D, if the sensor LED lights but `0x34` never changes, the limit port remap is
not enabled.** Send `9E 01`. This has caught two joints with an identical signature.
`EndLimit` is *not* the missing piece — it governs whether the driver acts on a limit,
not whether the pin is readable.

### 4. Move it

```bash
python3 .../joint_move.py z cw 15
python3 .../joint_move.py x ccw 30 --speed 10
python3 .../joint_move.py y cw 40 --dry-run
```

Three things to change: **joint, direction, distance.** Everything else — CAN ID, gear
ratio, which byte is clockwise, which IO bits are real, the safe speed ceiling — comes
from a table in the script, because each differs per joint and each has cost a session
when got wrong. A joint whose direction has never been confirmed by eye is **refused**
rather than guessed at.

Endstop and stall aborts are on by default. On an assembled arm add `--leave-coils on`
for any gravity-loaded joint: disabling the coils lets it fall.

### 5. Home it

Search for the limit, step clear of its detection zone, then zero:

```bash
python3 .../joint_move.py x cw 200          # watch for the endstop abort
python3 .../joint_move.py x cw 30           # step clear of the detection zone
cansend can0 001#9293                       # SET_AXIS_ZERO
```

**Do not use `GoHome` (`0x91`) on this arm.** Its direction lives in `Hm_Dir`, which
cannot be read back, and the manual says the motor runs until it hits the switch — so
if it runs the wrong way, or the watched input has no sensor, nothing stops it.

A homed zero **does not survive a power cycle.** The encoder counter resets at
power-up, so a joint reading `0` is not necessarily at home.

### 6. Measure its travel

```bash
python3 .../joint_calibrate.py z
python3 .../joint_calibrate.py z            # again -- one figure is not a result
```

Both limits are approached at the **same speed**, because approach speed biases the
trigger position from both ends at once and inflates the result. Run it twice: two runs
agreeing closely means the number is real. Joint Y measured 178.6° at speed 150, 165.5°
at 50, and 158.2° twice at 25 — the gears were slipping, and only the slowest pair
agreed.

### 7. Check the whole arm

```bash
python3 .../joint_roundtrip.py 45
python3 .../joint_roundtrip.py 10 --joints x,z
```

Each joint goes out and back; the report is how far from its start it finished. Joints
not on the bus, or without a confirmed direction, are skipped rather than failed.

---

## The thing that will mislead you most

**The encoder is on the motor shaft, upstream of the reduction.** Slipping gears produce
a flawless trace — delivered counts matching commanded to a fraction of a percent — for
an output that is going nowhere. Slip also *unloads* the motor, so no stall, torque or
position-error heuristic fires. The telemetry looks its best exactly when the drivetrain
has stopped transmitting.

This cost a full session on Joint Y: 150° of searching for an endstop that the arm never
approached, while every number on the bus looked perfect.

**No motion result here means anything without an observation of the output** — an
endstop trip, a physical reference, or somebody watching.

---

## Tools

All in `ros2_package/arctos_can_control/arctos_can_control/`.

**Everyday**

| Script | Purpose |
|---|---|
| `can_id_scan.py` | Read-only CAN ID discovery. Run first, always. |
| `joint_move.py` | Move one joint: joint, direction, distance. Endstop + stall aborts. |
| `joint_calibrate.py` | Endstop-to-endstop travel, both ends at one speed. |
| `joint_roundtrip.py` | Out-and-back across joints; reports return error. |

**Endstops**

| Script | Purpose |
|---|---|
| `joint_endstop_test.py` | Live read-only monitor of the IO byte. |
| `joint_endstop_dwell.py` | Timestamps each edge; says whether a trigger is solid or chattering. |
| `joint_io_diff.py` | Sweeps every read-only register with a magnet off then on — for when an endstop appears nowhere. |
| `joint_magnet_probe.py` | Whether an external magnet disturbs the encoder. |

**Diagnostics and history**

| Script | Purpose |
|---|---|
| `joint_reliability_test.py`, `joint_statistical_reliability_test.py` | Link quality and repeat-success rates. |
| `joint_micro_move_test.py` | Supervised first motion, typed confirmation required. |
| `joint_long_move_monitor.py` | Live trace of one large move. |
| `joint_speed_limit_test.py` | Speed ladder; finds where tracking degrades. |
| `joint_unstick_move_test.py`, `joint_sweep_unstick_test.py` | Back-off-and-retry for a gritty mesh. |
| `joint_stepped_move_test.py`, `joint_streamed_move_test.py`, `joint_position_control_test.py` | Earlier move strategies, kept for reference. |
| `mks_driver.py`, `joint_control_node.py`, `joint_state_publisher_node.py` | ROS 2 driver library and nodes. |

---

## Gripper firmware

[`firmware/arctos_gripper/`](firmware/arctos_gripper/) — a servo gripper as a CAN node.

```
PC -> CANable -> CAN bus -> MCP2515 -> SPI -> Arduino -> PWM -> servo
```

The MCP2515 is a CAN *controller*, not a microcontroller: it has no program of its own
and cannot transmit unaided. The Arduino is the piece that makes it a node, and it is
**not** connected to the PC in normal operation — USB is only for flashing.

```bash
cansend can0 007#02        # query position
cansend can0 007#0168      # set 104 degrees
cansend can0 007#03        # close
cansend can0 007#04        # open
candump can0               # replies arrive on 0x17, ending A5
```

Before flashing, set `MCP_CRYSTAL` to match the crystal can on your module (8 or
16 MHz). A mismatch produces no error — just a bitrate that is not 500 kbit/s and a
node that is silent or corrupts the bus. Most MCP2515 breakouts also carry their own
120 Ω terminator; check the 60 Ω reading after wiring it in.

**The travel limits in that sketch are measured, not nominal.** Commanding the servo
onto its mechanical stop was shown to take the node off the bus completely: 20 queries
parked clear of the stop returned 100%, parked on it returned 0%, and the command to
back off could not get through. A stalled servo holds full stall current, which sags
the shared 5 V rail the ATmega and MCP2515 both run from.

---

## Other directories

| Path | Contents |
|---|---|
| `gui/` | Tkinter control panel: bind the adapter to `can0`, enable/disable a motor, tear the interface down. Install with `bash gui/install_desktop_entry.sh`. |
| `firmware/*.bin` | CANable adapter images (slcan and candlelight/gs_usb). |
| `legacy/` | An early standalone script, superseded. Kept for history. |

---

## Requirements

- Ubuntu 22.04, ROS 2 Humble
- `python-can`, `can-utils`
- A CANable 2 or compatible SocketCAN adapter
- For the gripper: Arduino IDE or `arduino-cli`, with `mcp_can` and `Servo`

**If you are running this in a VM**, USB passthrough is a real source of trouble: the
CANable has been observed dropping off the bus, re-enumerating to a different
`ttyACM`, and staying enumerated while silently passing no traffic. Bare metal avoids
a class of problems that look like hardware faults.
