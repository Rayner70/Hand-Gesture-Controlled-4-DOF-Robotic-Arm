/**
 * @file    Glove_Esp01.ino
 * @author  Raphael Ebubechi Efita
 * @date    2024
 * @version 2.0
 *
 * @brief   Glove-Side ESP-01 Wi-Fi Bridge — Command Transmitter
 *
 * @details This firmware runs on the ESP-01 module mounted on the glove.
 *          It connects to the Wi-Fi Access Point hosted by the arm-side ESP-01,
 *          establishes a TCP connection to the arm server, and forwards every
 *          single-character command byte received from the Arduino Nano (via
 *          the hardware UART) to the arm unit over the wireless TCP link.
 *
 *          Data path:
 *            Nano (Glove_Arm.ino) ──UART──► This ESP-01 ──TCP/Wi-Fi──► Arm ESP-01
 *
 * @hardware
 *   - ESP-01 module (ESP8266, 3.3 V)
 *   - Receives command chars from Arduino Nano via hardware Serial (RX pin)
 *   - Transmits to arm-side ESP-01 Access Point over TCP (port 80)
 *
 * @dfm_notes
 *   Wi-Fi credentials and server address are in the CONFIGURATION section.
 *   Change only those constants to retarget this firmware to a different arm unit.
 *
 * @dependencies
 *   - ESP8266WiFi (built into ESP8266 Arduino core)
 *   - Board: "Generic ESP8266 Module" in Arduino IDE
 *
 * @license MIT
 */

// ─────────────────────────────────────────────────────────────────────────────
// DEPENDENCIES
// ─────────────────────────────────────────────────────────────────────────────
#include <ESP8266WiFi.h>


// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURATION
// ─────────────────────────────────────────────────────────────────────────────

/** @defgroup WiFiConfig  Arm-side Access Point credentials */
///@{
static const char* ARM_AP_SSID     = "ARM_WIFI";      ///< SSID of arm-side ESP-01 AP
static const char* ARM_AP_PASSWORD = "12345678";      ///< AP password
///@}

/** @defgroup ServerConfig  Arm server connection */
///@{
static const char*    ARM_SERVER_IP   = "192.168.4.1";  ///< Default ESP8266 AP IP
static const uint16_t ARM_SERVER_PORT = 80;             ///< TCP port on arm server
///@}

/** @defgroup SerialConfig  UART baud rate */
///@{
static const uint32_t SERIAL_BAUD = 9600;  ///< Must match Glove_Arm.ino espSerial baud
///@}


// ─────────────────────────────────────────────────────────────────────────────
// OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
WiFiClient client;  ///< TCP client connected to arm server


// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Connect to the arm Wi-Fi AP and establish initial TCP session.
 */
void setup() {
  Serial.begin(SERIAL_BAUD);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ARM_AP_SSID, ARM_AP_PASSWORD);

  Serial.print("Connecting to arm AP");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[OK] Connected to arm Wi-Fi.");
  Serial.print("[INFO] Local IP: ");
  Serial.println(WiFi.localIP());
}


// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Maintain TCP connection to arm server and forward incoming UART bytes.
 *
 * Re-connects automatically if the TCP session drops.
 * Any byte arriving from the Nano on UART RX is immediately forwarded to the arm.
 */
void loop() {
  // Ensure TCP connection is alive; reconnect if dropped.
  if (!client.connected()) {
    Serial.println("[RECONNECT] Connecting to arm server...");
    if (client.connect(ARM_SERVER_IP, ARM_SERVER_PORT)) {
      Serial.println("[OK] TCP connection established.");
    } else {
      Serial.println("[WARN] Connection failed. Retrying in 1s...");
      delay(1000);
      return;
    }
  }

  // Forward any byte received from the Nano straight to the arm server.
  if (Serial.available()) {
    char cmd = Serial.read();
    client.write(cmd);
  }
}
