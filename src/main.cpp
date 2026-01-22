#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "config.h"

// Hardware Serial 2 for ESP32 (RX=16, TX=17)
HardwareSerial LidarSerial(2);

#define LIDAR_BAUD 115200 
#define SYNC_BYTE 0xAA
#define MAX_PACKET_SIZE 128

rcl_publisher_t publisher;
sensor_msgs__msg__LaserScan scan_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float ranges[360];
bool agent_connected = false;
unsigned long total_packets_received = 0;
unsigned long successful_publishes = 0;

void setup() {
  Serial.begin(115200);
  delay(2000); 
  Serial.println("LIDAR Driver: Starting...");
  
  LidarSerial.setRxBufferSize(4096); 
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, 16, 17); 

  WiFi.begin(SECRET_SSID, SECRET_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  
  set_microros_wifi_transports((char*)SECRET_SSID, (char*)SECRET_PASS, (char*)SECRET_AGENT_IP, (size_t)SECRET_AGENT_PORT); 
  
  allocator = rcl_get_default_allocator();
  
  if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
    Serial.println("micro-ROS Support Init OK");
    if (rclc_node_init_default(&node, "droidal_lidar", "", &support) == RCL_RET_OK) {
      Serial.println("Node Init OK");
      
      // CHANGED: Using default (Reliable) publisher to fix the QoS Compatibility error found by ros2 doctor
      if (rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan") == RCL_RET_OK) {
        Serial.println("Publisher Init OK (Reliable Mode)");
        
        // Init Scan Msg structure
        scan_msg.header.frame_id.data = (char*)"laser_frame";
        scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
        scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;
        
        scan_msg.angle_min = 0;
        scan_msg.angle_max = 2.0 * M_PI;
        scan_msg.angle_increment = (2.0 * M_PI) / 360.0;
        scan_msg.range_min = 0.15;
        scan_msg.range_max = 8.0;
        
        scan_msg.ranges.data = ranges;
        scan_msg.ranges.size = 360;
        scan_msg.ranges.capacity = 360;
        
        for(int i=0; i<360; i++) ranges[i] = INFINITY;

        Serial.println("Syncing time...");
        rmw_uros_sync_session(1000); 
        
        delay(500);
        agent_connected = true;
      } else {
        Serial.println("Publisher Init FAILED");
      }
    }
  }

  Serial.println("Delta-2 DV005 Driver Ready.");
}

void processLidarByte(uint8_t b) {
  static uint8_t buffer[MAX_PACKET_SIZE];
  static int state = 0;
  static int count = 0;
  static int packet_len = 0;

  switch (state) {
    case 0: 
      if (b == 0xAA) {
        buffer[0] = b;
        state = 1;
      }
      break;
    case 1: 
      buffer[1] = b;
      state = 2;
      break;
    case 2: 
      buffer[2] = b;
      packet_len = 0x52; 
      count = 3;
      state = 3;
      break;
    case 3: 
      buffer[count++] = b;
      if (count >= (packet_len + 2)) { 
        total_packets_received++;
        
        uint8_t angle_index = buffer[11]; 
        float start_angle_deg = angle_index * 2.0f; 

        if (angle_index == 0) {
            for(int i=0; i<360; i++) ranges[i] = INFINITY;
        }

        for (int i = 0; i < 22; i++) {
          int offset = 12 + (i * 3);
          if (offset + 2 >= count) break;

          uint16_t dist_mm = (buffer[offset + 2] << 8) | buffer[offset + 1];
          float current_angle = fmod(start_angle_deg + (i * (2.0f / 22.0f)), 360.0f); 
          int angle_idx = (int)current_angle % 360;
          
          if (dist_mm > 150 && dist_mm < 8000) {
            ranges[angle_idx] = dist_mm / 1000.0f;
          }
        }
        state = 0;
      }
      break;
  }
}

void loop() {
  while (LidarSerial.available()) {
    processLidarByte(LidarSerial.read());
  }

  if (agent_connected) {
    static unsigned long last_pub = 0;
    if (millis() - last_pub > 100) { 
      int64_t ns = rmw_uros_epoch_nanos();
      
      scan_msg.header.stamp.sec = ns / 1000000000;
      scan_msg.header.stamp.nanosec = ns % 1000000000;

      scan_msg.ranges.data = ranges;
      scan_msg.ranges.size = 360;

      rcl_ret_t ret = rcl_publish(&publisher, &scan_msg, NULL);
      if (ret == RCL_RET_OK) {
        successful_publishes++;
      } else {
        static rcl_ret_t last_error = RCL_RET_OK;
        if (ret != last_error) {
          Serial.printf("Pub Error: %d\n", ret);
          last_error = ret;
        }
      }
      
      last_pub = millis();
      
      static unsigned long last_debug = 0;
      if (millis() - last_debug > 5000) {
        Serial.printf("Status: In=%lu, Out=%lu, WiFi=%d\n", 
                      total_packets_received, successful_publishes, WiFi.status());
        last_debug = millis();
      }

      static unsigned long last_sync = 0;
      if (millis() - last_sync > 30000) {
        rmw_uros_sync_session(10);
        last_sync = millis();
      }
    }
  }
}