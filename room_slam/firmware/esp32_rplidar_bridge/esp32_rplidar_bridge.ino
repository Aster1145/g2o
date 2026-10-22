/*
 * esp32_rplidar_bridge.ino
 * ---------------------------------------------------------------------------
 * Reads an RPLIDAR A1/A2/A3 on a hardware UART and streams each 360 degree
 * sweep to your laptop over WiFi, where slam2d/record.py stores it in SQLite.
 *
 *  lidar TX  -> ESP32 GPIO16 (RX2)      <-- only these two wires carry data
 *  lidar RX  -> ESP32 GPIO17 (TX2)
 *  lidar GND -> ESP32 GND
 *  lidar 5V  -> 5 V supply (NOT the 3V3 pin, and see docs/01_wiring.md)
 *  lidar MOTOCTL -> GPIO25 (or tie it to 5 V for full speed)
 *
 * Arduino IDE:  board = "ESP32 Dev Module", 115200 baud monitor.
 * PlatformIO :  see platformio.ini in this folder.
 *
 * WHAT IT SENDS (plain text, one message per line group):
 *
 *   SCAN <millis> <bins>
 *   0:1234 1:1230 2:0 ...          (16 "index:millimetres" pairs per line)
 *   END
 *
 *   0 = no return for that direction.  Everything above is parsed by
 *   room_slam/slam2d/record.py.
 */

#include <WiFi.h>
#include "rplidar_parser.h"

// ---------------------------------------------------------------------------
// 1. SET THESE FOUR VALUES
// ---------------------------------------------------------------------------
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* PC_HOST   = "192.168.1.50";   // your laptop's IP (see docs/03_network.md)
const uint16_t PC_PORT = 8888;            // must match: python -m slam2d.record

// ---------------------------------------------------------------------------
// 2. PINS  (change if your board wires the UARTs differently)
// ---------------------------------------------------------------------------
#define LIDAR_RX_PIN 16      // ESP32 receives lidar data here  <- lidar TX
#define LIDAR_TX_PIN 17      // ESP32 sends commands here      -> lidar RX
#define MOTOR_PIN    25      // lidar MOTOCTL (PWM). -1 = wired to 5 V instead
#define LIDAR_BAUD   115200

RplidarParser lidar(360);
WiFiClient client;
HardwareSerial& lidarPort = Serial2;

uint32_t scanCounter = 0;
uint32_t lastConnectAttempt = 0;
uint32_t lastStatus = 0;
uint32_t badStartResponses = 0;

// ---------------------------------------------------------------------------
// Lidar start-up
// ---------------------------------------------------------------------------
void startLidarMotor() {
  if (MOTOR_PIN >= 0) {
    pinMode(MOTOR_PIN, OUTPUT);
    analogWrite(MOTOR_PIN, 255);        // full speed, ~5.5 Hz -> 360 beams
  }
}

bool requestScan() {
  // STOP, then SCAN: makes the command idempotent if the ESP32 reboots
  // while the lidar keeps spinning.
  const uint8_t stopCmd[2] = {0xA5, 0x25};
  lidarPort.write(stopCmd, 2);
  delay(20);
  lidarPort.flush();

  const uint8_t scanCmd[2] = {0xA5, 0x20};
  lidarPort.write(scanCmd, 2);
  lidarPort.flush();

  // expected answer: A5 5A 05 00 00 40 81
  const uint8_t expect[7] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
  uint32_t deadline = millis() + 1000;
  int matched = 0;
  while (millis() < deadline) {
    if (!lidarPort.available()) continue;
    uint8_t b = (uint8_t)lidarPort.read();
    if (b == expect[matched]) {
      if (++matched == 7) {
        lidar.reset();
        lidar.clearBins();
        return true;
      }
    } else {
      matched = (b == expect[0]) ? 1 : 0;
    }
  }
  ++badStartResponses;
  return false;
}

void startLidar() {
  startLidarMotor();
  delay(300);                            // let the motor get up to speed
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (requestScan()) {
      Serial.println("# lidar: scan mode started");
      return;
    }
    Serial.println("# lidar: no answer, retrying");
    delay(300);
  }
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  static bool started = false;
  if (!started) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                // keeps latency low
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    started = true;
    Serial.printf("# wifi: connecting to %s\n", WIFI_SSID);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("# wifi: connected, ip=%s  laptop=%s:%u\n",
                  WiFi.localIP().toString().c_str(), PC_HOST, PC_PORT);
  }
}

void ensureClient() {
  if (client.connected()) return;
  uint32_t now = millis();
  if (now - lastConnectAttempt < 2000) return;
  lastConnectAttempt = now;
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.printf("# tcp: connecting to %s:%u ... ", PC_HOST, PC_PORT);
  if (client.connect(PC_HOST, PC_PORT, 1500)) {
    client.setNoDelay(true);
    client.printf("# esp32 rplidar bridge, %u bins\n", (unsigned)lidar.bins());
    Serial.println("connected");
  } else {
    Serial.println("failed (is record.py running? firewall?)");
  }
}

// ---------------------------------------------------------------------------
// Sending one revolution
// ---------------------------------------------------------------------------
void sendScan(const uint16_t* ranges, int bins, uint32_t stamp_ms) {
  if (!client.connected()) return;

  static char line[256];
  int len = snprintf(line, sizeof(line), "SCAN %u %d\n", (unsigned)stamp_ms, bins);
  client.write((const uint8_t*)line, len);

  int written = 0;                       // pairs per line
  line[0] = '\0';
  int pos = 0;
  for (int i = 0; i < bins; ++i) {
    pos += snprintf(line + pos, sizeof(line) - pos, "%d:%u ", i,
                    (unsigned)ranges[i]);
    if (++written == 16) {
      line[pos++] = '\n';
      client.write((const uint8_t*)line, pos);
      pos = 0;
      written = 0;
    }
    if (pos > (int)sizeof(line) - 24) {  // safety valve
      client.write((const uint8_t*)line, pos);
      pos = 0;
    }
  }
  if (pos > 0) {
    line[pos++] = '\n';
    client.write((const uint8_t*)line, pos);
  }
  client.print("END\n");
  ++scanCounter;
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n# esp32 rplidar bridge starting");

  lidarPort.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  startLidar();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void loop() {
  ensureWifi();
  ensureClient();

  // Drain the UART.  We always keep decoding, even while disconnected, so the
  // stream is in sync the moment the laptop comes back.
  int budget = 2048;
  while (lidarPort.available() && budget-- > 0) {
    uint8_t b = (uint8_t)lidarPort.read();
    if (!lidar.feed(b)) continue;

    if (client.connected()) {
      sendScan(lidar.ranges(), lidar.bins(), millis());
    }
    lidar.clearBins();
  }

  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    Serial.printf("# scans=%u beams=%d good=%u bad=%u wifi=%d client=%d heap=%u\n",
                  (unsigned)scanCounter, lidar.count(),
                  (unsigned)lidar.goodNodes(), (unsigned)lidar.badNodes(),
                  (int)WiFi.status(), (int)client.connected(),
                  (unsigned)ESP.getFreeHeap());
  }
}
