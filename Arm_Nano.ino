/**
 * @file    Arm_Nano.ino
 * @author  Raphael Ebubechi Efita
 * @date    2024
 * @version 2.0
 *
 * @brief   Arm Receiver — 4-DOF Servo Controller
 *
 * @details This firmware runs on the Arduino Nano mounted on the robotic arm.
 *          It receives single-character command tokens from the arm-side ESP-01
 *          (Arm_Esp01.ino) via SoftwareSerial, and drives four SG90 servo motors
 *          independently to move the corresponding arm joint.
 *
 *          Each joint tracks its own current position variable. On each command,
 *          only the targeted joint's position updates — all other joints remain
 *          unchanged at their last-known positions. Servos are only re-commanded
 *          if their target position actually changed, avoiding servo jitter.
 *
 *          Data path:
 *            Arm_Esp01.ino ──SoftwareSerial──► This Nano ──PWM──► 4× SG90 Servos
 *
 * @hardware
 *   - Arduino Nano (ATmega328P, 5 V)
 *   - ESP-01 Wi-Fi module → SoftwareSerial D10 (RX), D11 (TX)
 *   - Base servo     → D3  (PWM)
 *   - Shoulder servo → D5  (PWM)
 *   - Elbow servo    → D6  (PWM)
 *   - Gripper servo  → D9  (PWM)
 *
 * @command_map
 *   Char | Joint    | Direction
 *   ---- | -------- | ---------
 *   'A'  | Base     | Rotate LEFT  (−step)
 *   'B'  | Base     | Rotate RIGHT (+step)
 *   'C'  | Shoulder | Move UP      (+step)
 *   'D'  | Shoulder | Move DOWN    (−step)
 *   'E'  | Elbow    | Move UP      (+step)
 *   'F'  | Elbow    | Move DOWN    (−step)
 *   'G'  | Gripper  | OPEN         (+step)
 *   'H'  | Gripper  | CLOSE        (−step)
 *   "STATUS" | All | Report joint angles to Serial Monitor
 *
 * @dfm_notes
 *   STEP_SIZE, joint angle limits, and timing constants are all in the
 *   CONFIGURATION section. Adjust these without touching any logic functions.
 *
 * @license MIT
 */

// ─────────────────────────────────────────────────────────────────────────────
// DEPENDENCIES
// ─────────────────────────────────────────────────────────────────────────────
#include <SoftwareSerial.h>
#include <Servo.h>


// ─────────────────────────────────────────────────────────────────────────────
// HARDWARE PINS
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t PIN_ESP_RX  = 10;  ///< SoftwareSerial RX ← Arm_Esp01 TX
static const uint8_t PIN_ESP_TX  = 11;  ///< SoftwareSerial TX → Arm_Esp01 RX (unused)
static const uint8_t PIN_BASE    = 3;   ///< PWM → Base servo
static const uint8_t PIN_SHOULDER = 5;  ///< PWM → Shoulder servo
static const uint8_t PIN_ELBOW   = 6;   ///< PWM → Elbow servo
static const uint8_t PIN_GRIPPER = 9;   ///< PWM → Gripper servo


// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURATION  ← All tunable constants here. No logic edits needed.
// ─────────────────────────────────────────────────────────────────────────────

/** @defgroup JointConfig  Servo step size and angle limits */
///@{
static const int STEP_SIZE        = 10;   ///< Degrees per command (angular increment)

// Standard joints: 0–180° range
static const int JOINT_MIN_ANGLE  = 0;
static const int JOINT_MAX_ANGLE  = 180;

// Gripper has a narrower range to avoid mechanical damage
static const int GRIPPER_MIN_ANGLE = 30;
static const int GRIPPER_MAX_ANGLE = 120;
///@}

/** @defgroup InitConfig  Home positions on power-up */
///@{
static const int INIT_BASE     = 90;  ///< Base servo home (centred)
static const int INIT_SHOULDER = 90;  ///< Shoulder servo home
static const int INIT_ELBOW    = 90;  ///< Elbow servo home
static const int INIT_GRIPPER  = 60;  ///< Gripper servo home (partially closed)
///@}

/** @defgroup TimingConfig  Command debounce interval */
///@{
static const unsigned long COMMAND_DEBOUNCE_MS = 100;  ///< Min ms between command executions
///@}

/** @defgroup SerialConfig  Baud rates */
///@{
static const uint32_t SERIAL_BAUD     = 9600;  ///< USB debug monitor
static const uint32_t ESP_SERIAL_BAUD = 9600;  ///< SoftwareSerial from Arm_Esp01
///@}


// ─────────────────────────────────────────────────────────────────────────────
// OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
SoftwareSerial espSerial(PIN_ESP_RX, PIN_ESP_TX);

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;


// ─────────────────────────────────────────────────────────────────────────────
// GLOBAL STATE — current and last-known joint positions
// ─────────────────────────────────────────────────────────────────────────────
int basePos     = INIT_BASE;
int shoulderPos = INIT_SHOULDER;
int elbowPos    = INIT_ELBOW;
int gripperPos  = INIT_GRIPPER;

/// Shadow variables — servos are only re-commanded when position changes.
int lastBasePos     = INIT_BASE;
int lastShoulderPos = INIT_SHOULDER;
int lastElbowPos    = INIT_ELBOW;
int lastGripperPos  = INIT_GRIPPER;

String        commandBuffer  = "";   ///< Accumulates incoming chars until newline
unsigned long lastCommandTime = 0;   ///< Timestamp of last executed command


// ─────────────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────────────
void processCommand(const String& cmd);
void applyServoUpdates();
void printStatus();


// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Attach servos to PWM pins, home all joints, start serial interfaces.
 *
 * @note   Pulse width limits (1000–2000 µs) are specified explicitly in
 *         servo.attach() to ensure consistent behaviour across SG90 variants.
 */
void setup() {
  Serial.begin(SERIAL_BAUD);
  espSerial.begin(ESP_SERIAL_BAUD);

  delay(1000);

  // Attach with explicit pulse width limits for consistent SG90 behaviour.
  baseServo.attach(PIN_BASE,      1000, 2000);
  shoulderServo.attach(PIN_SHOULDER, 1000, 2000);
  elbowServo.attach(PIN_ELBOW,    1000, 2000);
  gripperServo.attach(PIN_GRIPPER, 1000, 2000);

  // Move all joints to home positions.
  baseServo.write(basePos);
  shoulderServo.write(shoulderPos);
  elbowServo.write(elbowPos);
  gripperServo.write(gripperPos);

  delay(500);
  Serial.println(F("[READY] Arm receiver active — 4-DOF independent control."));
  printStatus();
}


// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Read incoming bytes from ESP-01, buffer until newline, then process.
 */
void loop() {
  while (espSerial.available()) {
    char incoming = espSerial.read();

    if (incoming == '\r') {
      // Ignore carriage return
    } else if (incoming == '\n') {
      // End of command — process if non-empty
      if (commandBuffer.length() > 0) {
        processCommand(commandBuffer);
        commandBuffer  = "";
        lastCommandTime = millis();
      }
    } else {
      commandBuffer += incoming;
    }
  }
  delay(20);
}


// ─────────────────────────────────────────────────────────────────────────────
// COMMAND PROCESSOR
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Parse a command token and update the corresponding joint position.
 *
 * Only the target joint's variable is modified; all others stay at their
 * current values in memory. Servo writes are deferred to applyServoUpdates().
 *
 * @param cmd  Received command string (single char or "STATUS")
 */
void processCommand(const String& cmd) {
  // Debounce guard — ignore commands that arrive too fast.
  if (millis() - lastCommandTime < COMMAND_DEBOUNCE_MS) return;

  if (cmd == "A") {
    basePos = constrain(basePos - STEP_SIZE, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE);
    Serial.println(F("[CMD] Base LEFT"));
  }
  else if (cmd == "B") {
    basePos = constrain(basePos + STEP_SIZE, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE);
    Serial.println(F("[CMD] Base RIGHT"));
  }
  else if (cmd == "C") {
    shoulderPos = constrain(shoulderPos + STEP_SIZE, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE);
    Serial.println(F("[CMD] Shoulder UP"));
  }
  else if (cmd == "D") {
    shoulderPos = constrain(shoulderPos - STEP_SIZE, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE);
    Serial.println(F("[CMD] Shoulder DOWN"));
  }
  else if (cmd == "E") {
    elbowPos = constrain(elbowPos + STEP_SIZE, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE);
    Serial.println(F("[CMD] Elbow UP"));
  }
  else if (cmd == "F") {
    elbowPos = constrain(elbowPos - STEP_SIZE, JOINT_MIN_ANGLE, JOINT_MAX_ANGLE);
    Serial.println(F("[CMD] Elbow DOWN"));
  }
  else if (cmd == "G") {
    gripperPos = constrain(gripperPos + STEP_SIZE, GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE);
    Serial.println(F("[CMD] Gripper OPEN"));
  }
  else if (cmd == "H") {
    gripperPos = constrain(gripperPos - STEP_SIZE, GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE);
    Serial.println(F("[CMD] Gripper CLOSE"));
  }
  else if (cmd == "STATUS") {
    printStatus();
    return;  // No servo update needed for status query
  }
  else {
    // Unknown or empty token — ignore silently
    return;
  }

  applyServoUpdates();
}

/**
 * @brief  Write updated positions to servos only if their target changed.
 *
 * Comparing against shadow variables prevents unnecessary PWM updates,
 * which would cause servo jitter even when the arm is supposed to be still.
 */
void applyServoUpdates() {
  if (basePos != lastBasePos) {
    baseServo.write(basePos);
    lastBasePos = basePos;
  }
  if (shoulderPos != lastShoulderPos) {
    shoulderServo.write(shoulderPos);
    lastShoulderPos = shoulderPos;
  }
  if (elbowPos != lastElbowPos) {
    elbowServo.write(elbowPos);
    lastElbowPos = elbowPos;
  }
  if (gripperPos != lastGripperPos) {
    gripperServo.write(gripperPos);
    lastGripperPos = gripperPos;
  }
}

/**
 * @brief  Print all current joint angles to the USB Serial Monitor.
 */
void printStatus() {
  Serial.print(F("[STATUS] Base:"));     Serial.print(basePos);
  Serial.print(F("  Shoulder:"));        Serial.print(shoulderPos);
  Serial.print(F("  Elbow:"));           Serial.print(elbowPos);
  Serial.print(F("  Gripper:"));         Serial.println(gripperPos);
}
