# Arctos CAN Bus Integration — Bring-Up, Diagnostics, and Tooling

This repo collects the custom tooling, diagnostic scripts, and the living Standard
Operating Procedure (SOP) built while bringing up individual joints of an
[Arctos](https://github.com/Arctos-Robotics/ros2_arctos) robotic arm over CAN bus with
a CANable 2 adapter, on Ubuntu + ROS 2 Humble.

It does **not** include a copy of the upstream `ros2_arctos` repository itself — that
already exists as its own project with its own history. What's here is everything
built on top of it: joint-agnostic diagnostic scripts, a desktop control-panel app,
CANable firmware images, and the accumulated protocol/hardware findings.

## Start here

**[`ros2_package/arctos_can_control/arctos_can_sop.pdf`](ros2_package/arctos_can_control/arctos_can_sop.pdf)**
(source: `arctos_can_sop.tex` in the same folder) is the living SOP —
the actual detailed record of every finding, dead end, and fix across this project's
sessions. Read it before the code; the scripts assume the context it provides.

## Repository layout

```
ros2_package/arctos_can_control/   ROS 2 (ament_python) package: the diagnostic
                                    scripts, the ROS node, and the SOP itself.
docs/                               Reference documents assembled across sessions
                                    (an earlier/superseded Joint B guide and cheat
                                    sheet are kept for historical comparison — see
                                    the SOP's own "Protocol Errata" section for
                                    which claims in them turned out to be wrong).
gui/                                A small Tkinter desktop app with two buttons:
                                    bind the CANable to can0, and safely disable
                                    the motor + tear the interface down before
                                    unplugging it.
firmware/                           CANable adapter firmware images (slcan and
                                    candlelight/gs_usb variants) kept for reference.
legacy/                             An early standalone test script, superseded by
                                    the package's own scripts; kept for history.
```

## Diagnostic scripts (`ros2_package/arctos_can_control/arctos_can_control/`)

All of these are joint-agnostic — pass `--can-id`/`--gear-ratio` for whichever joint
is actually wired up, confirmed via `can_id_scan.py` first. None of them trust a
config file's or a driver panel's stated CAN ID; every session in this project found
at least one mismatch between what was assumed and what actually responded.

| Script | Purpose |
|---|---|
| `can_id_scan.py` | Read-only CAN ID discovery. Run this first, always. |
| `joint_reliability_test.py` | Read-only link-quality test (response rate, checksum integrity, encoder jitter) — no motor enable. |
| `joint_micro_move_test.py` | Supervised first-motion test, ~0.1° move, requires typed confirmation. |
| `joint_stepped_move_test.py` | Monitored multi-step move, absolute or relative, gated against a calibrated safe envelope. |
| `joint_streamed_move_test.py` | Sends a move as many rapid small increments instead of one large command. |
| `joint_position_control_test.py` | Uses `POSITION_CONTROL (0xFD)` — the opcode the project's own working setup script (`set_zero_position.py`) actually uses for repeated jogging, unlike the ROS 2 driver which only uses `0xF5`. |
| `joint_statistical_reliability_test.py` | N repetitions of one command, alternating or sustained-direction, to find real success rates and localize position-dependent issues rather than judging reliability from a single try. |
| `joint_long_move_monitor.py` | Live progress trace for a single large move — this is what revealed a stick-slip motion pattern that a short observation window would have missed entirely. |
| `mks_driver.py`, `joint_control_node.py`, `joint_state_publisher_node.py` | The ROS 2-facing driver library and nodes. |

## Selected findings (see the SOP for full detail and evidence)

- **`READ_ENCODER` is `0x31`, not `0x30`** — decoded as a signed 48-bit big-endian
  integer over 6 bytes (`degrees = raw48 * 360 / 16384`). An earlier draft used `0x30`,
  which isn't a defined command in the real driver at all.
- **Checksum is `(CAN_ID + sum(data_bytes)) & 0xFF`**, appended as the last byte —
  including on `EMERGENCY_STOP`. A bare, unchecksummed `F7` does not match what the
  real driver actually transmits.
- **A CAN driver's DIP switches or a config file's stated `motor_id` cannot be
  trusted** — verified three separate times across two joints where the real,
  responding CAN ID did not match either source.
- **A driver that responds to CAN reads/writes with zero communication errors can
  still be electrically silent** if it's the wrong hardware variant — an RS485
  version of a driver expected to be CAN produced clean silence (not bus errors)
  under every bitrate/ID combination tried.
- **`ENABLE_SHAFT_PROTECTION (0x88)` is required for `READ_SHAFT_PROTECTION_STATE
  (0x3E)` to detect and report a real stall** — without it, a genuine stall is
  indistinguishable from any other silent partial-motion result. It also produced at
  least one confirmed false-positive trip on a command that moved perfectly with it
  disabled, so treat a trip as a lead to investigate, not standalone proof.
- **A single large motion command and a sequence of small chunked commands can both
  hit the same underlying issue** — chunking a large move into small, individually-
  reliable steps does not by itself guarantee the full move completes; it can just
  relocate where a real, position-dependent mechanical issue is encountered.
- **A repeatable stick-slip motion pattern was traced to gear-mesh clearance**, not
  the downstream belt-driven load (confirmed by re-testing with the belt removed —
  the pattern was unchanged) and not a single localized defect (a second, distinct
  resistance point was found roughly 40° from the first). The working hypothesis is
  insufficient backlash in 3D-printed gears, which are being reprinted with more
  clearance.

## Desktop app

`gui/arctos_can_control_panel.py` — two buttons: "Set Up CAN" (binds the CANable
adapter to `can0` at 500 kbit/s) and "Disable Motor && Safe to Unplug" (scans for any
responding joint, disables it, then tears the interface down). Both use `pkexec` for
the one or two commands that actually need root, rather than requiring the app itself
to run privileged. Install it into your application menu with:

```bash
bash gui/install_desktop_entry.sh
```

## Requirements

- Ubuntu 22.04, ROS 2 Humble
- `python-can`, `can-utils`
- A CANable 2 (or compatible SocketCAN-capable) USB-to-CAN adapter
