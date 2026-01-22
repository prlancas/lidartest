#ifndef CONFIG_H
#define CONFIG_H

// WiFi Credentials
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ROS 2 Configuration
const char* AGENT_IP = "192.168.1.XX";
const size_t AGENT_PORT = 8888;

// OTA Configuration
const char* OTA_HOSTNAME = "ESP32-ODrive";
const char* OTA_PASS = "admin123";

#endif
