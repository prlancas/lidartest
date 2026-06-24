#include <WiFi.h>
#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "config.h"

// ---------------------------------------------------------------------------
//  Delta-2 LIDAR -> micro-ROS LaserScan publisher.
//
//  Data path (decoupled so WiFi/agent stalls never drop lidar bytes):
//    UART RX --(HW FIFO)--> LidarReadTask (high prio, pinned core) --> byte queue
//        --> loop(): parse packets (length + checksum validated) -> ranges[]
//        --> publish a LaserScan once per revolution (start-angle wrap).
//
//  Packet format: see LIDAR_PROTOCOL.md.  78-byte packets, 21 samples each:
//    [0]=AA [1..2]=len(BE)=total-2 [3]=01 [4]=61 [5]=AD(meas) [6..7]=datalen
//    [8]=rot_speed(*0.05 rev/s) [9..10]=FFDC [11..12]=start angle (0.01 deg)
//    [13..]=21 x {quality, dist_hi, dist_lo}  (distance big-endian mm)
//    [last2]=checksum(BE) = sum(all preceding bytes) & 0xFFFF
// ---------------------------------------------------------------------------

HardwareSerial LidarSerial(2);   // ESP32 UART2: RX=16, TX=17

#define LIDAR_BAUD       115200
#define SCAN_BINS        360     // LaserScan resolution (1 deg bins)
#define DEG_PER_SAMPLE   (22.5f / 21.0f)
#define RANGE_MIN_M      0.15f
#define RANGE_MAX_M      10.0f

// --- micro-ROS objects ---
rcl_publisher_t publisher;
sensor_msgs__msg__LaserScan scan_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float ranges[SCAN_BINS];
float intensities[SCAN_BINS];
bool  agent_connected = false;

// --- UART drain task / queue (keeps reading even while loop() is blocked) ---
static QueueHandle_t lidarQueue = NULL;
const size_t LIDAR_QUEUE_SIZE = 8192;   // ~0.7 s of buffer @ 115200 baud

// --- statistics (volatile: written in task, read in loop) ---
volatile uint32_t uartDropped   = 0;    // bytes the queue could not accept
uint32_t validPackets    = 0;
uint32_t checksumFails   = 0;
uint32_t bytesDiscarded  = 0;           // bytes thrown away while re-syncing
uint32_t publishCount    = 0;
float    lastRotation    = 0.0f;        // rev/s from most recent packet

// ---------------------------------------------------------------------------
//  UART reader task: drain the hardware UART into the software queue ASAP.
//  Pinned to core 1 at higher priority than the Arduino loop so it preempts
//  loop() (where blocking micro-ROS / WiFi work happens).
// ---------------------------------------------------------------------------
void LidarReadTask(void *pvParameters) {
  (void)pvParameters;
  uint8_t b;
  for (;;) {
    while (LidarSerial.available()) {
      b = (uint8_t)LidarSerial.read();
      if (xQueueSendToBack(lidarQueue, &b, 0) != pdTRUE) {
        uartDropped++;   // queue full: loop() is not draining fast enough
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------------------------------------------------------------------------
//  LaserScan publishing
// ---------------------------------------------------------------------------
void resetScan() {
  for (int i = 0; i < SCAN_BINS; i++) {
    ranges[i] = INFINITY;
    intensities[i] = 0.0f;
  }
}

void publishScan() {
  if (!agent_connected) return;

  int64_t ns = rmw_uros_epoch_nanos();
  scan_msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
  scan_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);

  // Timing derived from the measured rotation speed (guard against 0).
  float scan_time = (lastRotation > 0.1f) ? (1.0f / lastRotation) : 0.0f;
  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (float)SCAN_BINS;

  scan_msg.ranges.data = ranges;
  scan_msg.ranges.size = SCAN_BINS;
  scan_msg.intensities.data = intensities;
  scan_msg.intensities.size = SCAN_BINS;

  if (rcl_publish(&publisher, &scan_msg, NULL) == RCL_RET_OK) {
    publishCount++;
  }
}

// ---------------------------------------------------------------------------
//  Packet decode: fill ranges[]/intensities[] from one validated packet.
//  Publishes the completed scan when the start angle wraps (new revolution).
// ---------------------------------------------------------------------------
void decodePacket(const uint8_t *p, size_t total) {
  static float lastStartAngle = -1.0f;

  float startAngle = ((p[11] << 8) | p[12]) / 100.0f;   // 0.01 deg -> deg
  lastRotation = p[8] * 0.05f;                           // rev/s

  // A start angle that decreased means a new revolution began: the previous
  // scan is complete, so publish it and start a fresh one.
  if (lastStartAngle >= 0.0f && startAngle + 0.01f < lastStartAngle) {
    publishScan();
    resetScan();
  }
  lastStartAngle = startAngle;

  int nsamp = (int)((total - 15) / 3);                   // 21 for a 78-byte pkt
  for (int k = 0; k < nsamp; k++) {
    int o = 13 + k * 3;
    uint16_t dist_mm = (p[o + 1] << 8) | p[o + 2];       // big-endian, mm
    if (dist_mm == 0) continue;                          // no return

    float r = dist_mm / 1000.0f;
    if (r < RANGE_MIN_M || r > RANGE_MAX_M) continue;    // out of spec range

    float ang = fmodf(startAngle + k * DEG_PER_SAMPLE, 360.0f);
    int idx = (int)lroundf(ang);
    if (idx >= SCAN_BINS) idx -= SCAN_BINS;
    if (idx < 0) idx += SCAN_BINS;

    ranges[idx] = r;
    intensities[idx] = (float)p[o];                      // signal quality
  }
}

// ---------------------------------------------------------------------------
//  Streaming parser: scan the rolling buffer for complete, valid packets.
//  Re-syncs on the AA 00 .. 01 61 AD signature and validates the checksum,
//  so a single corrupt/dropped byte costs at most one packet, not a scan.
// ---------------------------------------------------------------------------
#define PKTBUF_SZ 1024
static uint8_t pktBuf[PKTBUF_SZ];
static size_t  pktLen = 0;

void tryParse() {
  size_t i = 0;
  while (true) {
    if (pktLen - i < 6) break;   // need the header signature to decide

    // Header: AA 00 <len_lo> 01 61 AD   (len high byte is 0x00)
    if (!(pktBuf[i] == 0xAA && pktBuf[i + 1] == 0x00 &&
          pktBuf[i + 3] == 0x01 && pktBuf[i + 4] == 0x61 &&
          pktBuf[i + 5] == 0xAD)) {
      i++; bytesDiscarded++;
      continue;
    }

    uint16_t frameLen = (pktBuf[i + 1] << 8) | pktBuf[i + 2];
    size_t total = (size_t)frameLen + 2;
    if (total < 17 || total > 256) {           // implausible length
      i++; bytesDiscarded++;
      continue;
    }
    if (pktLen - i < total) break;             // wait for the rest of the packet

    uint32_t sum = 0;
    for (size_t k = 0; k < total - 2; k++) sum += pktBuf[i + k];
    uint16_t ck = (pktBuf[i + total - 2] << 8) | pktBuf[i + total - 1];
    if ((sum & 0xFFFF) != ck) {                // bad checksum -> resync by 1
      checksumFails++; i++; bytesDiscarded++;
      continue;
    }

    decodePacket(pktBuf + i, total);
    validPackets++;
    i += total;
  }

  if (i > 0) {                                 // drop consumed/discarded bytes
    memmove(pktBuf, pktBuf + i, pktLen - i);
    pktLen -= i;
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("LIDAR Driver: Starting...");

  LidarSerial.setRxBufferSize(4096);
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, 16, 17);

  // Create the byte queue and start the UART drain task BEFORE the (slow)
  // WiFi / micro-ROS bring-up so the UART never overflows during startup.
  lidarQueue = xQueueCreate(LIDAR_QUEUE_SIZE, sizeof(uint8_t));
  if (lidarQueue == NULL) {
    Serial.println("FATAL: failed to create lidar queue");
    while (1) { delay(1000); }
  }
  if (xTaskCreatePinnedToCore(LidarReadTask, "Lidar_Read_Task",
                              3072, NULL, 3, NULL, 1) != pdPASS) {
    Serial.println("FATAL: failed to create LidarReadTask");
    while (1) { delay(1000); }
  }

  WiFi.begin(SECRET_SSID, SECRET_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

  set_microros_wifi_transports((char*)SECRET_SSID, (char*)SECRET_PASS,
                               (char*)SECRET_AGENT_IP, (size_t)SECRET_AGENT_PORT);

  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
    Serial.println("micro-ROS Support Init OK");
    if (rclc_node_init_default(&node, "droidal_lidar", "", &support) == RCL_RET_OK) {
      Serial.println("Node Init OK");

      if (rclc_publisher_init_default(
              &publisher, &node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
              "scan") == RCL_RET_OK) {
        Serial.println("Publisher Init OK (Reliable Mode)");

        scan_msg.header.frame_id.data = (char*)"laser_frame";
        scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
        scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;

        scan_msg.angle_min = 0.0f;
        scan_msg.angle_max = 2.0f * (float)M_PI;
        scan_msg.angle_increment = (2.0f * (float)M_PI) / (float)SCAN_BINS;
        scan_msg.range_min = RANGE_MIN_M;
        scan_msg.range_max = RANGE_MAX_M;
        scan_msg.scan_time = 0.0f;
        scan_msg.time_increment = 0.0f;

        scan_msg.ranges.data = ranges;
        scan_msg.ranges.size = SCAN_BINS;
        scan_msg.ranges.capacity = SCAN_BINS;
        scan_msg.intensities.data = intensities;
        scan_msg.intensities.size = SCAN_BINS;
        scan_msg.intensities.capacity = SCAN_BINS;

        resetScan();

        Serial.println("Syncing time...");
        rmw_uros_sync_session(1000);
        delay(500);
        agent_connected = true;
      } else {
        Serial.println("Publisher Init FAILED");
      }
    }
  }

  // Discard whatever piled up in the queue during startup so the stats below
  // reflect steady-state behaviour, not the WiFi/agent bring-up window.
  xQueueReset(lidarQueue);
  uartDropped = 0;

  Serial.println("Delta-2 Driver Ready.");
}

// ---------------------------------------------------------------------------
void loop() {
  // Drain the queue into the parse buffer and extract complete packets.
  // The UART keeps being read by the background task regardless of this loop.
  uint8_t b;
  int budget = 8192;
  while (budget-- > 0 && xQueueReceive(lidarQueue, &b, 0) == pdTRUE) {
    pktBuf[pktLen++] = b;
    if (pktLen >= PKTBUF_SZ - 1) tryParse();   // keep the buffer bounded
  }
  tryParse();

  static unsigned long last_debug = 0;
  if (millis() - last_debug > 5000) {
    Serial.printf("Pkts=%lu CkErr=%lu Disc=%lu UartDrop=%lu Pub=%lu "
                  "Q=%u Rot=%.2f Heap=%u\n",
                  (unsigned long)validPackets, (unsigned long)checksumFails,
                  (unsigned long)bytesDiscarded, (unsigned long)uartDropped,
                  (unsigned long)publishCount,
                  (unsigned)uxQueueMessagesWaiting(lidarQueue),
                  lastRotation, (unsigned)ESP.getFreeHeap());
    last_debug = millis();
  }

  static unsigned long last_sync = 0;
  if (agent_connected && millis() - last_sync > 30000) {
    rmw_uros_sync_session(10);
    last_sync = millis();
  }

  delay(1);   // brief yield; UART draining is handled by the background task
}
