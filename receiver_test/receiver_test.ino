#include <NowPair.h>

#define HEARTBEAT_MS 50
#define TIMEOUT_MS 150

NowPair peer_link(NowPair::Role::Slave, 6, 0x12345678, HEARTBEAT_MS, TIMEOUT_MS);

bool peerConnected = false;

void connectionChanged(bool connected) {
  peerConnected = connected;
  Serial.println(connected ? "Master connected" : "Master disconnected");
}

void received(const uint8_t *data, size_t length) {
  Serial.print("Master says: ");
  Serial.write(data, length);
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
  Serial.println("Listening for a master...");
}

void loop() {
  peer_link.update();

  static uint32_t lastMessage = 0;
  if (peer_link.connected() && uint32_t(millis() - lastMessage) >= 1000) {
    // Serial.println("connected!");
    const char message[] = "Hello from slave";
    
    Serial.println("data sent");
    if (peer_link.sendData(message, sizeof(message) - 1)) lastMessage = millis();
  }
  delay(1);
}

