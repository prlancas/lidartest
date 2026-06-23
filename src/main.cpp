#include <WiFi.h>
#include "config.h"

// Hardware Serial 2 for ESP32 (RX=16, TX=17)
HardwareSerial LidarSerial(2);

#define LIDAR_BAUD 115200

const char* SERVER_IP = "192.168.1.104";
const uint16_t SERVER_PORT = 8080;

WiFiClient tcpClient;
bool tcpConnected = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("LIDAR TCP Forwarder: Starting...");

  LidarSerial.setRxBufferSize(4096);
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, 16, 17);

  Serial.printf("Connecting to WiFi %s...\n", SECRET_SSID);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > 30000) {
      Serial.println("WiFi connect timeout, retrying...");
      start = millis();
    }
    delay(200);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

  Serial.printf("Attempting TCP connect to %s:%u\n", SERVER_IP, SERVER_PORT);
  tcpConnected = tcpClient.connect(SERVER_IP, SERVER_PORT);
  if (tcpConnected) Serial.println("TCP connected to server");
  else Serial.println("TCP connect failed, will retry in loop");

  Serial.println("Ready.");
}

void loop() {
  if (!tcpClient.connected()) {
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt > 2000) {
      Serial.println("Reconnecting TCP...");
      tcpConnected = tcpClient.connect(SERVER_IP, SERVER_PORT);
      if (tcpConnected) Serial.println("TCP reconnected");
      lastAttempt = millis();
    }
  }

  while (LidarSerial.available()) {
    uint8_t b = LidarSerial.read();
    if (tcpClient.connected()) {
      tcpClient.write(&b, 1);
    }
  }

  static unsigned long last_debug = 0;
  if (millis() - last_debug > 5000) {
    Serial.printf("WiFi=%d, TCP=%d, FreeHeap=%u\n", WiFi.status(), tcpClient.connected(), ESP.getFreeHeap());
    last_debug = millis();
  }
}