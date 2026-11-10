#include <NowPair.h>
#include "IBusBM.h"
#include "../common/packets.h"

#define NETWORK_ID 0x12345678
#define HEARTBEAT_MS 50
#define TIMEOUT_MS 250

NowPair peer_link(NowPair::Role::Master, 6, NETWORK_ID, HEARTBEAT_MS, TIMEOUT_MS);
bool peerConnected = false;

IBusBM IBus;

void connectionChanged(bool connected, void *arg) {
  peerConnected = connected;
  Serial.println(connected ? "Slave connected" : "Slave disconnected");
}

void received(const uint8_t *data, size_t length,int rssiDbm, int rssiTimestamp, void *arg) {
  TelemetryPacket packet;
  bool res = packet.deserialize(data, length);
  if (res) {
    Serial.printf("x:%1.3f,y:%.3f,z:%.3f,rpm:%.3f\n", TelemetryPacket::convertToG(packet.a_x), TelemetryPacket::convertToG(packet.a_y), TelemetryPacket::convertToG(packet.a_z), packet.rpm);
    // Serial.printf("x:%d,y:%d,z:%d\n", packet.a_x, packet.a_y, packet.a_z);
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    Serial.println("bad packet!");
  }
  // Serial.print("Slave says: ");
  // Serial.write(data, length); // Payload is not necessarily null-terminated.
  // Serial.println();
}

void setup() {
  Serial.begin(115200);

  IBus.begin(Serial0, IBUSBM_NOTIMER);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // needed before any networking or else there is a slight disconnect at beginning
  delay(1000);

  peer_link.onConnectionChange(connectionChanged, NULL);
  peer_link.onData(received, NULL);
  if (!peer_link.begin()) {
    Serial.println("ESP-NOW initialization failed");
    while (true) delay(1000);
  }
  Serial.println("Broadcasting discovery; waiting for slave...");
}

int last_cnt_rec_update = 0;
int last_cnt_rec = 0;

void loop() {
  static uint8_t data_buffer[228];
  ThrottlePacket packet;

  // transmit at 1khz
  int time_between_packets = 19;
  digitalWrite(LED_BUILTIN, HIGH);

  peer_link.update();  // Must run frequently, including while disconnected.
  IBus.loop();

  // 0 - right, left-right
  // 1 - right, up-down
  // 2 - left, left-right
  // 3 - left, up-down
  // 4 - vra
  // 5 - vrb
  packet.right_lr = IBus.readChannel(0) - 1000;
  packet.right_ud = IBus.readChannel(1) - 1000;

  packet.left_lr = IBus.readChannel(3) - 1000;
  packet.left_ud = IBus.readChannel(2) - 1000;

  packet.knob_left = IBus.readChannel(4) - 1000;
  packet.knob_right = IBus.readChannel(5) - 1000;

  // Serial.printf("swa:%d,swb:%d,swc:%d,swd:%d\n",IBus.readChannel(6),IBus.readChannel(7),IBus.readChannel(8),IBus.readChannel(9));

  packet.swa = (IBus.readChannel(6) - 1000) > 500;
  packet.swb = (IBus.readChannel(7) - 1000) > 500;
  // packet.swc = IBus.readChannel(4) - 1000;
  // packet.swd = IBus.readChannel(5) - 1000;

  // assume connected, updated later
  packet.connected = true;

  // TODO: on controller dc, sometimes toggles between frozen values / failsafe values for a couple secs, then goes back to normal

  // two failsafe methods:
  // - check if no signal (through cnt_rec)
  // - check if controller is not connected (through failsafe values)

  // failsafe values
  // NOTE: MUST KEEP UPDATED AT ALL TIMES
  uint16_t fail_l_lr = 177;
  uint16_t fail_l_ud = 2;
  uint16_t fail_r_lr = 690;
  uint16_t fail_r_ud = 327;
  uint16_t fail_kl = 255;
  uint16_t fail_kr = 794;

  int fail_cnt = 0;
  if (fail_l_lr == packet.left_lr) fail_cnt++;
  if (fail_l_ud == packet.left_ud) fail_cnt++;
  if (fail_r_lr == packet.right_lr) fail_cnt++;
  if (fail_r_ud == packet.right_ud) fail_cnt++;
  if (fail_kl == packet.knob_left) fail_cnt++;
  if (fail_kr == packet.knob_right) fail_cnt++;

  if (fail_cnt >= 5) {
    // more than likely a disconnect, indicate such
    packet.connected = false;
  }

  if (IBus.cnt_rec != last_cnt_rec) {
    last_cnt_rec = IBus.cnt_rec;
    last_cnt_rec_update = millis();
  }

  // way too long without any new packets, disconnect
  if (millis() - last_cnt_rec_update > 100) {
    packet.connected = false;
  }

  static uint32_t lastMessage = 0;

  if (peer_link.connected() && uint32_t(millis() - lastMessage) >= time_between_packets) {
    size_t n = packet.serialize(data_buffer);
    if (peer_link.sendData(data_buffer, n)) lastMessage = millis();
  } else {
  }

  delay(10);
}
