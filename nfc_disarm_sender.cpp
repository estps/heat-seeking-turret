#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Reed sensor wiring
#define REED_PIN    3   // ESP32-C3 GPIO3 (change as needed)

// ESPNOW peer MAC address (replace with your turret ESP32 MAC)
uint8_t turretAddress[] = {0x64, 0xB7, 0x08, 0x29, 0x48, 0x50};

// ESPNOW message struct
typedef struct {
  char msg[8];
} message_t;

message_t disarmMsg = {"DISARM"};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optionally print status
}

void setup() {
  Serial.begin(115200);
  Serial.println("Reed Switch Disarm Sender (ESP32-C3) - Startup DISARM");

  pinMode(REED_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (1);
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, turretAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1);
  }

  // Send DISARM immediately on startup
  esp_now_send(turretAddress, (uint8_t *)&disarmMsg, sizeof(disarmMsg));
  Serial.println("DISARM sent on startup.");
}

void loop() {
  // Nothing to do in loop
}