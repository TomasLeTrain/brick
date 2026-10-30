#include <NowPair.h>

#define HEARTBEAT_MS 50
#define TIMEOUT_MS 150

NowPair peer_link(NowPair::Role::Master, 6, 0x12345678, HEARTBEAT_MS, TIMEOUT_MS);
bool peerConnected = false;

void connectionChanged(bool connected) {
  peerConnected = connected;
  Serial.println(connected ? "Slave connected" : "Slave disconnected");
}

void received(const uint8_t *data, size_t length) {
  Serial.print("Slave says: ");
  Serial.write(data, length); // Payload is not necessarily null-terminated.
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  peer_link.onConnectionChange(connectionChanged);
  peer_link.onData(received);
  if (!peer_link.begin()) {
    Serial.println("ESP-NOW initialization failed");
    while (true) delay(1000);
  }
  Serial.println("Broadcasting discovery; waiting for slave...");
}

void loop() {
  peer_link.update(); // Must run frequently, including while disconnected.

  static uint32_t lastMessage = 0;
  if (peer_link.connected() && uint32_t(millis() - lastMessage) >= 1000) {
    // Serial.println("connected!");
    const char message[] = "Hello from master";
    Serial.println("data sent");
    if (peer_link.sendData(message, sizeof(message) - 1)) lastMessage = millis();
  }
  delay(1);
}

