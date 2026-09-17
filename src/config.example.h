#ifndef CONFIG_H
#define CONFIG_H

// WiFi Credentials
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// TCP forwarder configuration
// Host failover is selected in main.cpp: 192.168.1.165, then 192.168.1.104.
const size_t AGENT_PORT = 8080;

// OTA Configuration
const char* OTA_HOSTNAME = "ESP32-ODrive";
const char* OTA_PASS = "admin123";

#endif
