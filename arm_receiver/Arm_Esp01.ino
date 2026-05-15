/**
 * @file    Arm_Esp01.ino
 * @author  Raphael Ebubechi Efita
 * @date    2024
 * @version 2.0
 *
 * @brief   Arm-Side ESP-01 Wi-Fi Bridge — Access Point & Command Receiver
 *
 * @details This firmware runs on the ESP-01 module mounted on the robotic arm.
 *          It creates a Wi-Fi Access Point that the glove-side ESP-01 connects to,
 *          listens for incoming TCP connections, and forwards every received
 *          command byte to the Arduino Nano (ARM_NANO.ino) via UART.
 *
 *          Data path:
 *            Glove ESP-01 ──TCP/Wi-Fi──► This ESP-01 (AP) ──UART──► Arm Nano
 *
 * @hardware
 *   - ESP-01 module (ESP8266, 3.3 V)
 *   - Creates Wi-Fi AP that glove ESP-01 connects to
 *   - Forwards received commands to Arduino Nano via hardware Serial (TX pin)
 *
 * @dfm_notes
 *   AP credentials and server port are in the CONFIGURATION section.
 *   ARM_AP_SSID and ARM_AP_PASSWORD must match what is set in Glove_Esp01.ino.
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

/** @defgroup APConfig  This device's Access Point settings */
///@{
static const char*    ARM_AP_SSID     = "ARM_WIFI";   ///< Must match Glove_Esp01.ino
static const char*    ARM_AP_PASSWORD = "12345678";   ///< Must match Glove_Esp01.ino
static const uint16_t SERVER_PORT     = 80;           ///< TCP port to listen on
///@}

/** @defgroup SerialConfig  UART to Nano */
///@{
static const uint32_t SERIAL_BAUD = 9600;  ///< Must match ARM_NANO.ino espSerial baud
///@}


// ─────────────────────────────────────────────────────────────────────────────
// OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
WiFiServer server(SERVER_PORT);  ///< TCP server awaiting glove ESP-01 connection
WiFiClient client;               ///< Connected glove client handle


// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Start the Wi-Fi Access Point and begin listening for TCP connections.
 */
void setup() {
  Serial.begin(SERIAL_BAUD);

  WiFi.softAP(ARM_AP_SSID, ARM_AP_PASSWORD);
  server.begin();

  Serial.println("[OK] Arm Wi-Fi AP started.");
  Serial.print("[INFO] AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("[WAIT] Waiting for glove connection...");
}


// ─────────────────────────────────────────────────────────────────────────────
// MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief  Accept incoming glove connection and relay command bytes to Nano.
 *
 * If a byte is received from the glove ESP-01 over TCP, it is immediately
 * written to the UART (Serial) which the Arduino Nano reads on its espSerial port.
 */
void loop() {
  // Accept new connection if one arrives and none is active.
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("[OK] Glove connected.");
    }
    return;
  }

  // Forward every incoming TCP byte directly to the Nano UART.
  if (client.available()) {
    char cmd = client.read();
    Serial.write(cmd);
  }
}
