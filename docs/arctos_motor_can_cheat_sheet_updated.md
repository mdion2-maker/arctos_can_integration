# 🤖 Arctos Motor + CAN Bus Cheat Sheet

**Joint B:** MKS Servo42D  
**CAN ID:** `0x05`  
**CAN interface:** `can0`  
**Bitrate:** `500000` (500 kbit/s)

> **Last verified:** August 19, 2026  
> **Purpose:** Practical CAN/ROS 2 reference for Joint B while debugging reliable motion.

---

# 🚨 SAFETY FIRST

If Joint B is hot, unexpectedly holding torque, or behaving unexpectedly:

### Disable the motor

```bash
cansend can0 005#F300
```

`F3 00` = motor disable.

### Emergency stop

```bash
cansend can0 005#F7
```

`F7` = emergency stop.

**Do not continue motion testing while the motor is heating rapidly.** Keep the joint clear and be ready to disable it immediately.

---

# 🔌 CAN INTERFACE

## Check CAN status

```bash
ip -details link show can0
```

Expected configuration:

```text
CAN state ERROR-ACTIVE
bitrate 500000
```

## Bring CAN down

```bash
sudo ip link set can0 down
```

## Bring CAN up at 500 kbit/s

```bash
sudo ip link set can0 up type can bitrate 500000
```

## Restart CAN

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

---

# 👀 MONITOR CAN TRAFFIC

## Everything on the bus

```bash
candump can0
```

Stop with `Ctrl+C`.

## Joint B only

```bash
candump can0,005:7FF
```

Joint B uses standard CAN ID `0x05`.

## Log traffic to a file

```bash
candump -L can0 > can_log.txt
```

---

# 🧮 CAN CHECKSUM

For the commands tested manually, the final byte is the checksum:

```text
checksum = (CAN_ID + all preceding data bytes) & 0xFF
```

For Joint B:

```text
CAN_ID = 0x05
```

Examples:

```text
31 + 05 = 36
→ 31 36
```

```text
3A + 05 = 3F
→ 3A 3F
```

```text
F1 + 05 = F6
→ F1 F6
```

```text
F3 + 01 + 05 = F9
→ F3 01 F9
```

> **Important:** Do not invent checksums for motion commands. Use the checksum calculation implemented in the existing Python/C++ protocol code.

---

# 🎛️ MOTOR ENABLE / DISABLE

Your source defines:

```cpp
ENABLE_MOTOR = 0xF3
```

## Enable

```bash
cansend can0 005#F301F9
```

Payload:

```text
F3 01 F9
```

## Disable

```bash
cansend can0 005#F300
```

Payload:

```text
F3 00
```

The source constructs `disableMotor()` using:

```cpp
{
    CANCommands::ENABLE_MOTOR,
    0x00
}
```

The source also defines `enableMotor()` using:

```cpp
{
    CANCommands::ENABLE_MOTOR,
    0x01
}
```

---

# 🛑 EMERGENCY STOP

Your source defines:

```cpp
EMERGENCY_STOP = 0xF7
```

Send:

```bash
cansend can0 005#F7
```

`F7` is the emergency-stop command.

### Do not confuse these

| Command | Meaning |
|---|---|
| `F7` | Emergency stop |
| `F300` | Disable motor |
| `F301F9` | Enable motor |

---

# 📡 VERIFIED STATUS / READ COMMANDS

The project source defines:

```cpp
READ_ENCODER = 0x31
READ_VELOCITY = 0x32
READ_ERROR = 0x39
READ_ENABLE_STATE = 0x3A
READ_SHAFT_PROTECTION_STATE = 0x3E
```

## Read encoder

```bash
cansend can0 005#3136
```

Observed request:

```text
005#31 36
```

Latest observed response:

```text
005#31 00 00 00 00 00 03 39
```

**Keep the raw response in the log when analyzing encoder behavior.** The project has conflicting encoder-scale assumptions, so do not reduce this response to a single “count” without first confirming the exact Servo42D response format.

---

## Read velocity

The source defines:

```cpp
READ_VELOCITY = 0x32
```

For the previously observed command pattern:

```bash
cansend can0 005#3237
```

Keep the complete response from `candump`; do not infer a velocity value unless the response format is confirmed.

---

## Read error

```bash
cansend can0 005#393E
```

Observed request:

```text
005#39 3E
```

Recent observed traffic included:

```text
005#39 FF FF FF F6 31
```

Treat the four-byte value as signed only after confirming the protocol's byte layout. The raw CAN response is the authoritative measurement.

---

## Read enable state

The source defines:

```cpp
READ_ENABLE_STATE = 0x3A
```

For Joint B, the checksum request is:

```bash
cansend can0 005#3A3F
```

because:

```text
0x05 + 0x3A = 0x3F
```

Recent traffic also showed:

```text
005#3A42
005#3A00 43
```

These frames demonstrate command/response traffic, but **do not label a particular returned byte as enabled/disabled unless it has been matched to the exact protocol response format.**

---

## Read shaft-protection state

The source defines:

```cpp
READ_SHAFT_PROTECTION_STATE = 0x3E
```

Request:

```bash
cansend can0 005#3E43
```

because:

```text
0x05 + 0x3E = 0x43
```

Observed traffic included:

```text
005#3E43
005#3E00 43
```

---

# 🔎 MOTOR QUERY / STATUS

Your source defines:

```cpp
QUERY_MOTOR = 0xF1
```

Request:

```bash
cansend can0 005#F1F6
```

because:

```text
0x05 + 0xF1 = 0xF6
```

Observed response:

```text
005#F1 01 F7
```

This confirms that the motor responds to the query.

---

# 🧪 VERIFIED CAN TEST RESULTS — AUGUST 19, 2026

## Communication

Joint B has repeatedly responded on CAN ID `0x05` to:

```text
F1
39
3A
3E
31
```

and has returned multi-byte responses.

**Conclusion:** CAN communication with Joint B is working.

## Motor query

Request:

```text
F1 F6
```

Response:

```text
F1 01 F7
```

This is a verified motor-query response.

## Encoder request

Request:

```text
31 36
```

Response:

```text
31 00 00 00 00 00 03 39
```

Retain this as the latest raw encoder response until its byte layout is confirmed.

## Enable/status observations

Observed traffic includes:

```text
F3 01 F9
F3 01 F9
```

showing that the enable command is being acknowledged/echoed on the bus.

Also observed:

```text
3A 42
3A 00 43
```

and:

```text
3E 43
3E 00 43
```

These are useful for diagnosing enable and shaft-protection state.

---

# 📊 CURRENT VERIFIED STATE

| Parameter | Current information |
|---|---|
| Motor | MKS Servo42D |
| Joint | B |
| CAN ID | `0x05` |
| Interface | `can0` |
| Bitrate | `500000` |
| CAN communication | ✅ Working |
| Motor query (`F1`) | ✅ Responding |
| Encoder request (`31`) | ✅ Responding |
| Error request (`39`) | ✅ Responding |
| Enable-state request (`3A`) | ✅ Responding |
| Shaft-protection request (`3E`) | ✅ Responding |
| Enable command (`F3 01`) | ✅ Observed acknowledged/echoed |
| Reliable motion | ❌ Not yet demonstrated |
| Main investigation | Working mode + exact motion packet construction |

### Important conclusion

**The CAN bus is not the primary failure.**

Joint B is communicating and responding to multiple read/control commands.

The remaining problem is to determine why a correctly addressed and communicating motor does not reliably execute the desired motion command.

---

# 🧭 MOTOR WORKING MODE

Your source defines:

```cpp
enum class MotorMode {
    CR_OPEN  = 0,
    CR_CLOSE = 1,
    CR_vFOC  = 2,
    SR_OPEN  = 3,
    SR_CLOSE = 4,
    SR_vFOC  = 5
};
```

The command is:

```cpp
SET_WORKING_MODE = 0x82
```

The project previously used a default:

```text
working_mode = 2
```

which corresponds to:

```text
CR_vFOC
```

**Do not change the motor's working mode blindly.** Confirm the intended Servo42D mode and exact `0x82` payload before writing a new setting.

---

# 📍 MOTION COMMANDS

Your source defines:

```cpp
RELATIVE_POSITION = 0xF4
ABSOLUTE_POSITION = 0xF5
SPEED_CONTROL = 0xF6
POSITION_CONTROL = 0xFD
ABSOLUTE_POSITION_PULSE = 0xFE
```

| Command | Hex | Purpose |
|---|---:|---|
| Relative position | `0xF4` | Relative movement |
| Absolute position | `0xF5` | Absolute movement |
| Speed control | `0xF6` | Speed control |
| Position control | `0xFD` | Position control |
| Absolute position pulse | `0xFE` | Absolute pulse position |

---

# ⚠️ MOTION COMMAND WARNING

A previously transmitted frame was:

```text
F5 00 64 14 00 30 23 C5
```

Do **not** treat this as a known-good movement command. The motor did not move during that test.

Before another motion test:

1. Keep `candump` running.
2. Verify the joint is mechanically safe to move.
3. Use the smallest practical movement.
4. Monitor temperature.
5. Record the exact transmitted frame.
6. Record the complete motor response.
7. Read the encoder before and after.
8. Stop immediately if the motor heats rapidly.

---

# 📐 ENCODER SCALE — IMPORTANT DISCREPANCY

Two encoder scales exist in the project/testing history.

### C++ constants

```text
ENCODER_STEPS = 0x4000
              = 16384 counts/revolution
```

Therefore:

```text
16384 / 360 ≈ 45.51 counts/degree
```

### Earlier Python calibration

```text
COUNTS_PER_REV = 16050
```

Therefore:

```text
16050 / 360 ≈ 44.58 counts/degree
```

### Status

These values **do not match**.

Do not use either value as the final joint-angle conversion until the encoder response format and calibration are reconciled.

---

# 🔍 SOURCE CODE SEARCHES

Go to the driver:

```bash
cd ~/arctos_ws/src/ros2_arctos/arctos_motor_driver
```

## Find motor enable code

```bash
grep -Rni -C 25 "ENABLE_MOTOR" .
```

## Find absolute-position code

```bash
grep -Rni -C 25 "ABSOLUTE_POSITION" .
```

## Find encoder code

```bash
grep -Rni -C 25 "READ_ENCODER" .
```

## Find working-mode code

```bash
grep -Rni -C 25 "working_mode" .
```

## Find a hexadecimal command

```bash
grep -Rni "0xF3" ./src
```

## Find position-command construction

```bash
grep -Rni -C 25 "setJointPosition" .
```

## Find CAN frame transmission

```bash
grep -Rni -C 25 "sendFrame" .
```

---

# 📁 IMPORTANT PROJECT FILES

## Command definitions

```bash
nano ~/arctos_ws/src/ros2_arctos/arctos_motor_driver/include/arctos_motor_driver/motor_types.hpp
```

## Main driver

```bash
nano ~/arctos_ws/src/ros2_arctos/arctos_motor_driver/src/motor_driver.cpp
```

## Python motor script

```bash
nano ~/arctos_ws/src/ros2_arctos/scripts/stepper_motor/move_motor.py
```

---

# 🏗️ BUILD THE ROS 2 DRIVER

```bash
cd ~/arctos_ws
colcon build --packages-select arctos_motor_driver
```

Then:

```bash
source ~/arctos_ws/install/setup.bash
```

---

# 🐍 PYTHON CAN

Check python-can:

```bash
python3 -c "import can; print(can.__version__)"
```

Modern python-can uses:

```python
interface="socketcan"
```

instead of the deprecated:

```python
bustype="socketcan"
```

---

# 🤖 ROS 2 QUICK COMMANDS

## Nodes

```bash
ros2 node list
```

## Topics

```bash
ros2 topic list
```

## Topic data

```bash
ros2 topic echo /topic_name
```

## Services

```bash
ros2 service list
```

## Actions

```bash
ros2 action list
```

---

# 📊 CAN ERROR DIAGNOSTICS

```bash
ip -details -statistics link show can0
```

Check:

```text
CAN state
RX errors
TX errors
dropped packets
bus errors
```

A healthy interface should not accumulate CAN errors during normal testing.

---

# 🧪 SAFE TEST WORKFLOW

Use this sequence when continuing Joint B testing.

### 1. Check CAN

```bash
ip -details link show can0
```

### 2. Start the monitor

```bash
candump can0,005:7FF
```

### 3. Query motor

```bash
cansend can0 005#F1F6
```

### 4. Read encoder

```bash
cansend can0 005#3136
```

### 5. Read error

```bash
cansend can0 005#393E
```

### 6. Read enable state

```bash
cansend can0 005#3A3F
```

### 7. If mechanically safe, enable

```bash
cansend can0 005#F301F9
```

### 8. Perform only a very small motion test

Use a motion frame whose byte layout and checksum have been verified first.

### 9. Immediately read encoder again

```bash
cansend can0 005#3136
```

### 10. Disable after testing

```bash
cansend can0 005#F300
```

---

# ⭐ QUICK REFERENCE

### 🔴 Disable motor

```bash
cansend can0 005#F300
```

### 🛑 Emergency stop

```bash
cansend can0 005#F7
```

### 🟢 Enable motor

```bash
cansend can0 005#F301F9
```

### 👀 Monitor Joint B

```bash
candump can0,005:7FF
```

### 🔍 Read encoder

```bash
cansend can0 005#3136
```

### 🔍 Read error

```bash
cansend can0 005#393E
```

### 🔍 Read enable state

```bash
cansend can0 005#3A3F
```

### 🔍 Query motor

```bash
cansend can0 005#F1F6
```

### 🔌 Check CAN

```bash
ip -details link show can0
```

### 🔴 CAN down

```bash
sudo ip link set can0 down
```

### 🔄 Restart CAN

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

---

# 🧠 CURRENT DEBUGGING CONCLUSION

```text
CAN interface             ✅
500 kbit/s communication  ✅
Motor responds            ✅
Motor query               ✅
Encoder read              ✅
Error read                ✅
Enable-state read         ✅
Shaft-protection read     ✅
Enable command            ✅
Reliable motion            ❌

Next investigation:
1. Confirm exact working mode.
2. Confirm exact F4/F5 packet layout.
3. Confirm checksum generation.
4. Send the smallest safe motion command.
5. Compare encoder before/after.
6. Monitor temperature and motor status.
```

**Do not keep repeating enable commands as a substitute for testing motion.** The communication path is already demonstrated. The useful next data is the exact motion frame, its response, encoder change, and motor status.
