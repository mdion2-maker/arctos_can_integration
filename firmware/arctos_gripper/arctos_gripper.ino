/*
 * arctos_gripper.ino
 *
 * End-effector servo as a CAN node on the Arctos arm.
 *
 * The MCP2515 is a CAN controller, not a microcontroller: it has no program of
 * its own and cannot put anything on the bus unaided. This sketch is the piece
 * that drives it over SPI and turns received frames into servo movement, so the
 * chain is
 *
 *     PC -> CANable -> CAN bus -> MCP2515 -> SPI -> this Arduino -> PWM -> servo
 *
 * The Arduino is NOT connected to the PC in normal operation. USB is only for
 * flashing this sketch; afterwards it runs from the step-down converter.
 *
 * ---------------------------------------------------------------- protocol --
 * Deliberately NOT the MKS protocol, so a stray gripper frame can never be
 * mistaken for a joint reply by the Python tooling.
 *
 *   set angle (PC -> gripper)   ID 0x07   [0]=0x01  [1]=angle (clamped)
 *   query     (PC -> gripper)   ID 0x07   [0]=0x02
 *   close     (PC -> gripper)   ID 0x07   [0]=0x03   to the stop, then back off
 *   open      (PC -> gripper)   ID 0x07   [0]=0x04
 *   reply     (gripper -> PC)   ID 0x17   [0]=0x01  [1]=angle now  [2]=0xA5
 *
 * Byte [2] of the reply is a fixed signature, so a listener can tell a real
 * gripper reply from a coincidence.
 *
 *     cansend can0 007#0105A      -> move to 90 degrees  (0x5A = 90)
 *     cansend can0 007#02         -> ask where it is
 *
 * ----------------------------------------------------------------- wiring --
 *   MCP2515        Arduino (Uno/Nano)      Arduino Mega 2560
 *     VCC            5V                      5V
 *     GND            GND                     GND
 *     CS             D10 (MCP_CS_PIN)        D53 -- change MCP_CS_PIN below
 *     SO  (MISO)     D12                     D50
 *     SI  (MOSI)     D11                     D51
 *     SCK            D13                     D52
 *     INT            D2  (optional)          D2
 *
 *   servo signal -> SERVO_PIN
 *   servo V+     -> step-down output, NOT the Arduino 5V pin
 *   servo GND    -> step-down GND, tied to Arduino GND (common ground REQUIRED,
 *                   or the servo sees no valid signal)
 *
 * A servo under load draws far more than the Arduino's regulator can supply.
 * Powering it from the 5V pin browns out the board and causes resets that look
 * like random CAN dropouts.
 *
 * ---------------------------------------------------------- before flashing --
 *  1. MCP_CRYSTAL below MUST match the crystal can on your MCP2515 module.
 *     Read the number printed on the silver can: 8.000 means MCP_8MHZ,
 *     16.000 means MCP_16MHZ. Getting this wrong does not give an error -- it
 *     gives a bitrate that is not 500k, and the node is simply silent or
 *     corrupts the bus.
 *  2. Most MCP2515 breakout boards carry their own 120 ohm terminator. The bus
 *     must have exactly TWO. With power off, measure CAN_H to CAN_L: 60 ohm is
 *     correct, 40 ohm means this module added a third and it must be removed.
 */

#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>

// ------------------------------------------------------------- configuration
const uint8_t  MCP_CS_PIN      = 10;      // D53 on a Mega 2560
const uint8_t  SERVO_PIN       = 9;
const uint16_t GRIPPER_CAN_ID  = 0x07;    // free: joints occupy 0x01..0x06
const uint16_t GRIPPER_REPLY_ID = 0x17;

#define MCP_CRYSTAL MCP_16MHZ             // <-- MCP_8MHZ or MCP_16MHZ. CHECK IT.

// ---- MECHANICAL LIMITS -- MEASURED ON THE MACHINE 2026-10-23 -------------
// The servo is coupled to the gripper and cannot reach a hobby servo's nominal
// 0..180. Its real travel is 30 degrees, found by stepping outward from the
// closed stop and stopping when the jaws would go no further.
//
// Driving past these stalls the servo, which then draws full stall current
// continuously. On this shared 5V rail that sags the supply the ATmega and the
// MCP2515 both depend on, so the damage shows up as CAN comms faults rather
// than as anything obviously mechanical. That is exactly what happened while
// these numbers were still guesses.
const uint8_t CLOSED_STOP_DEG = 90;    // jaws fully closed, hard stop
const uint8_t OPEN_STOP_DEG   = 120;   // jaws fully open, hard stop

// Stay this far inside the open stop in normal use. Nothing is gained by
// pressing against it, and resting there is a stall held indefinitely.
const uint8_t OPEN_MARGIN_DEG = 2;

// After a deliberate close, retreat this far. Gripping an object IS a stall by
// design, so holding the grip means holding stall current unless we back off.
// Kept small because the whole range is only 30 degrees.
const uint8_t BACKOFF_DEG = 3;

const uint8_t SERVO_START_DEG = CLOSED_STOP_DEG + BACKOFF_DEG;

// ------------------------------------------------------------------ protocol
const uint8_t CMD_SET   = 0x01;
const uint8_t CMD_QUERY = 0x02;
const uint8_t CMD_CLOSE = 0x03;   // go to the closed stop, then back off
const uint8_t CMD_OPEN  = 0x04;   // go to the open stop
const uint8_t REPLY_SIGNATURE = 0xA5;

// Usable band: the closed stop is reachable because gripping needs it, but the
// open end stops short of its stop. Written without assuming which end is
// numerically higher, so the limits cannot be silently inverted later.
uint8_t openTarget() {
  return (OPEN_STOP_DEG > CLOSED_STOP_DEG)
       ? (uint8_t)(OPEN_STOP_DEG - OPEN_MARGIN_DEG)
       : (uint8_t)(OPEN_STOP_DEG + OPEN_MARGIN_DEG);
}

uint8_t clampToTravel(int16_t deg) {
  const int16_t a = CLOSED_STOP_DEG;
  const int16_t b = openTarget();
  const int16_t lo = (a < b) ? a : b;
  const int16_t hi = (a < b) ? b : a;
  if (deg < lo) return (uint8_t)lo;
  if (deg > hi) return (uint8_t)hi;
  return (uint8_t)deg;
}

// One step back from the closed stop, toward open, whichever way that is.
uint8_t restingClosedAngle() {
  const int16_t dir = (OPEN_STOP_DEG > CLOSED_STOP_DEG) ? 1 : -1;
  return clampToTravel((int16_t)CLOSED_STOP_DEG + dir * (int16_t)BACKOFF_DEG);
}

MCP_CAN CAN(MCP_CS_PIN);
Servo gripper;
uint8_t currentAngle = SERVO_START_DEG;

void sendReply() {
  uint8_t buf[3] = { CMD_SET, currentAngle, REPLY_SIGNATURE };
  CAN.sendMsgBuf(GRIPPER_REPLY_ID, 0, 3, buf);
}

void setup() {
  gripper.attach(SERVO_PIN);
  gripper.write(currentAngle);

  // Retry rather than give up: on a cold start the MCP2515 can need a moment,
  // and a node that silently fails to init is indistinguishable from one that
  // is not wired in at all.
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_CRYSTAL) != CAN_OK) {
    delay(1000);
  }
  CAN.setMode(MCP_NORMAL);   // without this the controller stays in listen-only
}

void loop() {
  if (CAN.checkReceive() != CAN_MSGAVAIL) return;

  unsigned long rxId;
  uint8_t len = 0;
  uint8_t buf[8];
  CAN.readMsgBuf(&rxId, &len, buf);

  if (rxId != GRIPPER_CAN_ID || len < 1) return;

  switch (buf[0]) {
    case CMD_SET:
      if (len >= 2) {
        currentAngle = clampToTravel((int16_t)buf[1]);
        gripper.write(currentAngle);
      }
      sendReply();
      break;

    case CMD_CLOSE:
      // Drive to the stop, give it time to arrive, then retreat. Sitting on the
      // stop is a stall; this holds the grip without holding stall current.
      gripper.write(CLOSED_STOP_DEG);
      delay(400);
      currentAngle = restingClosedAngle();
      gripper.write(currentAngle);
      sendReply();
      break;

    case CMD_OPEN:
      currentAngle = openTarget();
      gripper.write(currentAngle);
      sendReply();
      break;

    case CMD_QUERY:
      sendReply();
      break;

    default:
      break;   // not ours; stay quiet rather than add noise to the bus
  }
}
