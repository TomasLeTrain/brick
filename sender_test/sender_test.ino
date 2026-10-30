#include <NowPair.h>
#include "../common/packets.h"

#define HEARTBEAT_MS 50
#define TIMEOUT_MS 150

NowPair peer_link(NowPair::Role::Master, 6, 0x12345678, HEARTBEAT_MS, TIMEOUT_MS);
bool peerConnected = false;

void connectionChanged(bool connected, void * arg) {
  peerConnected = connected;
  Serial.println(connected ? "Slave connected" : "Slave disconnected");
}

void received(const uint8_t *data, size_t length, void * arg) {
  TelemetryPacket packet;
  bool res = packet.deserialize(data, length);
  if(res){
    Serial.printf("x:%.3f,y:%.3f,z:%.3f\n", packet.a_x, packet.a_y, packet.a_z);
    digitalWrite(LED_BUILTIN, LOW);
    // Serial.printf("received data %d\n", micros());
  }else{
    Serial.println("bad packet!");
  }
  // Serial.print("Slave says: ");
  // Serial.write(data, length); // Payload is not necessarily null-terminated.
  // Serial.println();
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // needed before any networking or else there is a slight disconnect at beginning
  delay(1000);

  peer_link.onConnectionChange(connectionChanged,NULL);
  peer_link.onData(received,NULL);
  if (!peer_link.begin()) {
    Serial.println("ESP-NOW initialization failed");
    while (true) delay(1000);
  }
  Serial.println("Broadcasting discovery; waiting for slave...");
}

void loop() {
  peer_link.update(); // Must run frequently, including while disconnected.

  static uint8_t data_buffer[228];
  ThrottlePacket packet;

  int time_between_packets = 20;
  
  digitalWrite(LED_BUILTIN, HIGH);

  static uint32_t lastMessage = 0;
  if (peer_link.connected() && uint32_t(millis() - lastMessage) >= time_between_packets) {
    packet.throttle = 12345;
    size_t n = packet.serialize(data_buffer);

    // Serial.printf("sent data: %d\n", micros());
    if (peer_link.sendData(data_buffer, n)) lastMessage = millis();
  }else{

  }
  delay(1);
}

