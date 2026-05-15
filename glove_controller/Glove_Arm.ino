/**
 * @file    Glove_Arm.ino
 * @author  Raphael Ebubechi Efita
 * @date    2024
 * @version 2.0
 *
 * @brief   Glove Controller — Hand Gesture to Robotic Arm Command Transmitter
 *
 * @details This firmware runs on an Arduino Nano mounted on the operator's glove.
 *          It reads motion data from an MPU6050 IMU (accelerometer + gyroscope),
 *          interprets specific hand gestures as 4-DOF robotic arm commands, and
 *          forwards single-character command tokens to an ESP-01 Wi-Fi module
 *          via SoftwareSerial for wireless transmission to the arm receiver.
 *
 *          Two MPU6050 libraries are used in tandem for complementary data:
 *            - Adafruit MPU6050 : Raw accelerometer readings (tilt detection)
 *            - MPU6050_light    : Fused angle calculations (yaw/roll via gyro integration)
 *
 *          A 5-sample moving average filter is applied to the Z-axis accelerometer
 *          reading to suppress vibration noise on the elbow axis.
 *
 * @hardware
 *   - Arduino Nano (ATmega328P, 5 V)
 *   - MPU6050 IMU (I2C: SDA=A4, SCL=A5)
 *   - ESP-01 Wi-Fi Module → SoftwareSerial D10 (RX), D11 (TX)
 *
 * @gesture_map
 *   Gesture                 | Axis Used         | Command Sent
 *   ----------------------- | ----------------- | ------------
 *   Tilt wrist forward      | Accel Y +         | 'C' (Shoulder UP)
 *   Tilt wrist backward     | Accel Y −         | 'D' (Shoulder DOWN)
 *   Tilt hand left          | Accel X −         | 'A' (Base LEFT)
 *   Tilt hand right         | Accel X +         | 'B' (Base RIGHT)
 *   Lift palm upward        | Accel Z +         | 'E' (Elbow UP)
 *   Lower palm downward     | Accel Z −         | 'F' (Elbow DOWN)
 *   Twist wrist clockwise   | Yaw angle +       | 'G' (Gripper OPEN)
 *   Twist wrist counter-CW  | Yaw angle −       | 'H' (Gripper CLOSE)
 *   Neutral / no gesture    | Within dead zone  | '' (STOP)
 *
 * @dfm_notes
 *   All threshold and timing constants are in the CONFIGURATION section below.
 *   To recalibrate for a different operator or glove fit, only that section
 *   needs editing. No logic changes required.
 *
 * @dependencies
 *   - Adafruit MPU6050 library (Library Manager: "Adafruit MPU6050")
 *   - Adafruit Unified Sensor (dependency of Adafruit MPU6050)
 *   - MPU6050_light library (Library Manager: "MPU6050_light")
 *
 * @license MIT
 */

// ─────────────────────────────────────────────────────────────────────────────
// DEPENDENCIES
// ─────────────────────────────────────────────────────────────────────────────
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>   ///< Raw accelerometer + gyro readings
#include <Adafruit_Sensor.h>    ///< Adafruit unified sensor abstraction layer
#include <MPU6050_light.h>      ///< Fused angle calculations (yaw/roll/pitch)


// ─────────────────────────────────────────────────────────────────────────────
// HARDWARE PINS
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t PIN_ESP_RX = 10;  ///< SoftwareSerial RX from ESP-01 TX
static const uint8_t PIN_ESP_TX = 11;  ///< SoftwareSerial TX to ESP-01 RX


// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURATION  ← Adjust all thresholds here; do not edit logic functions.
// ─────────────────────────────────────────────────────────────────────────────

/** @defgroup ShoulderConfig  Shoulder joint thresholds (Accel Y-axis, m/s²) */
///@{
static const float SHOULDER_UP_THRESHOLD   =  5.0f;
static const float SHOULDER_DOWN_THRESHOLD = -5.0f;
///@}

/** @defgroup BaseConfig  Base rotation thresholds (Accel X-axis, m/s²) */
///@{
static const float BASE_LEFT_THRESHOLD  = -5.0f;
static const float BASE_RIGHT_THRESHOLD =  7.0f;
///@}

/** @defgroup ElbowConfig  Elbow thresholds (Accel Z-axis, m/s² — noise-filtered) */
///@{
static const float ELBOW_UP_THRESHOLD   =  10.0f;
static const float ELBOW_DOWN_THRESHOLD =  8.0f;
///@}

/** @defgroup GripperConfig  Gripper thresholds (integrated Yaw angle, degrees) */
///@{
static const float GRIPPER_OPEN_THRESHOLD  =  20.0f;   ///< CW  wrist twist
static const float GRIPPER_CLOSE_THRESHOLD = -15.0f;   ///< CCW wrist twist
///@}

/** @defgroup FilterConfig  Moving average filter for elbow Z-axis */
///@{
static const int   FILTER_SIZE     = 5;    ///< Number of samples in the rolling window
///@}

/** @defgroup TimingConfig  Command send rate */
///@{
static const unsigned long SEND_INTERVAL_MS = 150;  ///< Minimum ms between command transmissions
///@}

/** @defgroup SerialConfig  Baud rates */
///@{
static const uint32_t SERIAL_BAUD     = 9600;  ///< USB serial (debug monitor)
static const uint32_t ESP_SERIAL_BAUD = 9600;  ///< SoftwareSerial to ESP-01
///@}


// ─────────────────────────────────────────────────────────────────────────────
// OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
SoftwareSerial espSerial(PIN_ESP_RX, PIN_ESP_TX);  ///< UART bridge to ESP-01 module
Adafruit_MPU6050 adaMPU;                           ///< Raw accelerometer readings
MPU6050          lightMPU(Wire);                   ///< Fused angle calculations


// ─────────────────────────────────────────────────────────────────────────────
// GLOBAL STATE
// ─────────────────────────────────────────────────────────────────────────────
String        lastCommand    = "";        ///< Previously sent command (avoids redundant sends)
unsigned long lastSendTime   = 0;         ///< Timestamp of last transmission (ms)

/// Moving average filter state for Z-axis
float zAxisBuffer[FILTER_SIZE] = {0};
int   bufferIndex              = 0;


// ─────────────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────────────
String determineCommand(float ax, float ay, float az, float roll, float yaw);
float  getFilteredZAxis(float rawZ);
void   sendArmCommand(const String& cmd);
void   logDebug(const String& msg);


// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Initialise serial ports, I2C bus, and both MPU6050 instances.
 *
 * @note   Gyro offset calibration (calcGyroOffsets) runs at startup.
 *         The glove must be held STILL for ~2 seconds during power-on.
 *         This is critical for accurate yaw integration.
 */
void setup() {
  Serial.begin(SERIAL_BAUD);
  espSerial.begin(ESP_SERIAL_BAUD);
  Wire.begin();

  // --- Adafruit MPU6050 (raw accel) ---
  if (!adaMPU.begin()) {
    Serial.println(F("[ERROR] Adafruit MPU6050 not found. Check wiring."));
    while (1);
  }
  adaMPU.setAccelerometerRange(MPU6050_RANGE_8_G);
  adaMPU.setGyroRange(MPU6050_RANGE_500_DEG);
  adaMPU.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("[OK] Adafruit MPU6050 ready."));

  // --- MPU6050_light (fused angles) ---
  lightMPU.begin();
  Serial.println(F("[CAL] Hold glove STILL — calculating gyro offsets..."));
  lightMPU.calcGyroOffsets();
  Serial.println(F("[OK] Gyro offsets calibrated."));

  delay(500);
  Serial.println(F("[READY] Glove controller active."));
  Serial.println(F("  Y-axis tilt  → Shoulder | X-axis tilt → Base"));
  Serial.println(F("  Z-axis lift  → Elbow    | Wrist twist → Gripper"));
}


// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Continuously read IMU data, classify gesture, and transmit command.
 */
void loop() {
  // Update fused angle calculations (must be called frequently for accurate integration)
  lightMPU.update();

  // Read raw accelerometer from Adafruit library
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  adaMPU.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  // Retrieve fused angles from MPU6050_light
  float yawAngle  = lightMPU.getAngleZ();
  float rollAngle = lightMPU.getAngleY();

  // Classify the current hand pose into a command token
  String command = determineCommand(
    accelEvent.acceleration.x,
    accelEvent.acceleration.y,
    accelEvent.acceleration.z,
    rollAngle,
    yawAngle
  );

  // Transmit if command changed or heartbeat interval elapsed
  bool commandChanged = (command != lastCommand);
  bool intervalElapsed = (millis() - lastSendTime > SEND_INTERVAL_MS);

  if (commandChanged || intervalElapsed) {
    sendArmCommand(command);
    lastCommand  = command;
    lastSendTime = millis();
  }

  delay(50);  ///< ~20 Hz loop rate; keep ≥ 50ms for I2C stability
}


// ─────────────────────────────────────────────────────────────────────────────
// GESTURE CLASSIFICATION
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Map raw IMU readings to a single-character arm command token.
 *
 * Priority order: Shoulder → Base → Elbow → Gripper → Stop.
 * Only one DOF is controlled at a time (highest-priority gesture wins).
 *
 * @param ax    Raw accelerometer X (m/s²)
 * @param ay    Raw accelerometer Y (m/s²)
 * @param az    Raw accelerometer Z (m/s²) — passed through moving average filter
 * @param roll  Fused roll angle (degrees) — reserved for future use
 * @param yaw   Fused yaw angle (degrees) — used for gripper twist detection
 *
 * @return  Single-char command string: 'A'–'H' or "" (stop)
 */
String determineCommand(float ax, float ay, float az, float roll, float yaw) {

  // --- Shoulder (Y-axis tilt: forward/backward wrist bend) ---
  if      (ay >  SHOULDER_UP_THRESHOLD)   return "C";  // Shoulder UP
  else if (ay <  SHOULDER_DOWN_THRESHOLD) return "D";  // Shoulder DOWN

  // --- Base rotation (X-axis tilt: hand lean left/right) ---
  else if (ax <  BASE_LEFT_THRESHOLD)     return "A";  // Base LEFT
  else if (ax >  BASE_RIGHT_THRESHOLD)    return "B";  // Base RIGHT

  // --- Elbow (filtered Z-axis: palm up/down) ---
  float filteredZ = getFilteredZAxis(az);
  if      (filteredZ >  ELBOW_UP_THRESHOLD)   return "E";  // Elbow UP
  else if (filteredZ < -ELBOW_DOWN_THRESHOLD) return "F";  // Elbow DOWN

  // --- Gripper (yaw angle: wrist rotation CW/CCW) ---
  else if (yaw >  GRIPPER_OPEN_THRESHOLD)  return "G";  // Gripper OPEN
  else if (yaw <  GRIPPER_CLOSE_THRESHOLD) return "H";  // Gripper CLOSE

  // --- Neutral zone: no gesture detected ---
  return "";
}


// ─────────────────────────────────────────────────────────────────────────────
// HELPER FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Apply a 5-sample circular moving average to the Z-axis reading.
 *
 * Suppresses high-frequency vibration noise from arm/glove mechanical coupling.
 * The filter window size is controlled by FILTER_SIZE in the CONFIGURATION section.
 *
 * @param  rawZ  Latest Z-axis accelerometer reading (m/s²)
 * @return Smoothed Z-axis value
 */
float getFilteredZAxis(float rawZ) {
  zAxisBuffer[bufferIndex] = rawZ;
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

  float sum = 0.0f;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += zAxisBuffer[i];
  }
  return sum / FILTER_SIZE;
}

/**
 * @brief  Transmit a command token to the ESP-01 module and log to serial monitor.
 *
 * @param cmd  Command string to send (empty string = stop)
 */
void sendArmCommand(const String& cmd) {
  espSerial.println(cmd);

  if (cmd.length() > 0) {
    Serial.print(F("[TX] → "));
    Serial.println(cmd);
  }
}

/**
 * @brief  Print a timestamped debug message to the USB serial monitor.
 * @param msg  Message to print
 */
void logDebug(const String& msg) {
  Serial.print(F("["));
  Serial.print(millis());
  Serial.print(F(" ms] "));
  Serial.println(msg);
}
