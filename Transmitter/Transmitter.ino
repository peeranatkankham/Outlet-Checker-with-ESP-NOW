#include <esp_now.h>
#include <WiFi.h>

// Define a unique ID for each device
#define DEVICE_ID 1  // Change for each device: 1, 2, or 3
#define HEARTBEAT_INTERVAL 1000  // 1 seconds

// Structure to hold the message data
typedef struct {
  int deviceId;
  long timestamp;
} heartbeat_message_t;

// Define the receiver's MAC address (make sure to replace it with your receiver's MAC address)
uint8_t receiverMac[] = {0x30, 0xAE, 0xA4, 0xFF, 0xFF, 0xFF}; // Replace with your receiver's MAC address

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  // Add the receiver's MAC address
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  // Create a heartbeat message
  heartbeat_message_t heartbeat;
  heartbeat.deviceId = DEVICE_ID;
  heartbeat.timestamp = millis();

  // Send the heartbeat message
  esp_now_send(receiverMac, (uint8_t*)&heartbeat, sizeof(heartbeat));

  // Wait for the defined interval before sending the next heartbeat
  delay(HEARTBEAT_INTERVAL);
}