#include <WiFi.h>
#include <Arduino.h>
#include "config.h"

// Hardware Serial 2 for ESP32 (RX=16, TX=17)
HardwareSerial LidarSerial(2);

#define LIDAR_BAUD 115200

// Prefer the local Docker host, then fall back to the existing robot host.
// The LiDAR stream must reach whichever machine is currently running rosconfig.
const char* const SERVER_IPS[] = {"192.168.1.165", "192.168.1.104"};
const size_t SERVER_IP_COUNT = sizeof(SERVER_IPS) / sizeof(SERVER_IPS[0]);
const uint16_t SERVER_PORT = 8080;

WiFiClient tcpClient;

// FreeRTOS queue to hold incoming bytes from LiDAR
static QueueHandle_t lidarQueue = NULL;
const size_t LIDAR_QUEUE_SIZE = 4096; // number of bytes

bool connectToServer() {
  for (size_t i = 0; i < SERVER_IP_COUNT; ++i) {
    const char* serverIp = SERVER_IPS[i];
    Serial.printf("Attempting TCP connect to %s:%u\\n", serverIp, SERVER_PORT);
    tcpClient.stop();
    if (tcpClient.connect(serverIp, SERVER_PORT)) {
      Serial.printf("TCP connected to %s\\n", serverIp);
      return true;
    }
  }
  Serial.println("TCP connect failed on both configured hosts");
  return false;
}

// Task that continuously drains the hardware UART into the software queue
void LidarReadTask(void * pvParameters) {
  (void)pvParameters;
  uint8_t b;
  for (;;) {
    while (LidarSerial.available()) {
      b = LidarSerial.read();
      // Non-blocking: drop oldest if queue full by using 0 tick timeout
      xQueueSendToBack(lidarQueue, &b, 0);
    }
    // Yield to other tasks briefly
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("LIDAR TCP Forwarder: Starting...");

  LidarSerial.setRxBufferSize(4096);
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, 16, 17);

  // Create queue
  lidarQueue = xQueueCreate(LIDAR_QUEUE_SIZE, sizeof(uint8_t));
  if (lidarQueue == NULL) {
    Serial.println("Failed to create LiDAR queue");
    while (1) { delay(1000); }
  }

  // Start background task pinned to core 1 to keep UART drained
  BaseType_t t = xTaskCreatePinnedToCore(
    LidarReadTask,
    "Lidar_Read_Task",
    3072,
    NULL,
    3,
    NULL,
    1
  );
  if (t != pdPASS) {
    Serial.println("Failed to create LidarReadTask");
  }

  Serial.printf("Connecting to WiFi %s...\n", SECRET_SSID);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > 30000) {
      Serial.println("WiFi connect timeout, retrying...");
      start = millis();
    }
    delay(200);
    Serial.print('.');
  }
  Serial.println("\nWiFi Connected");

  if (!connectToServer()) Serial.println("Will retry in loop");

  Serial.println("Ready.");
}

void loop() {
  static unsigned long lastAttempt = 0;
  if (!tcpClient.connected()) {
    if (millis() - lastAttempt > 2000) {
      Serial.println("Reconnecting TCP (trying both hosts)...");
      connectToServer();
      lastAttempt = millis();
    }
  }

  // Drain up to a chunk from the queue and send in one write
  const size_t CHUNK_SZ = 512;
  uint8_t buf[CHUNK_SZ];
  size_t idx = 0;
  while (idx < CHUNK_SZ) {
    uint8_t v;
    if (xQueueReceive(lidarQueue, &v, 0) == pdTRUE) {
      buf[idx++] = v;
    } else break;
  }
  if (idx > 0 && tcpClient.connected()) {
    tcpClient.write(buf, idx);
  }

  static unsigned long last_debug = 0;
  if (millis() - last_debug > 5000) {
    Serial.printf("WiFi=%d, TCP=%d, Queue=%u, FreeHeap=%u\n", WiFi.status(), tcpClient.connected(), uxQueueMessagesWaiting(lidarQueue), ESP.getFreeHeap());
    last_debug = millis();
  }

  // Small delay to avoid busy-looping
  delay(1);
}
