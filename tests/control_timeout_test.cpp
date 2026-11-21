#include "../brick/src/subsystems/comms.h"
#include <LittleFS.h>
#include <cassert>
#include <map>
uint32_t host_ms = 1000;
bool host_motion = false;
unsigned host_flash_operations = 0;
HostSerial Serial;
HostLittleFS LittleFS;
std::map<std::string, std::vector<uint8_t>> host_files;
const uint8_t remote[6] = {2, 0, 0, 0, 0, 1}, local[6] = {2, 0, 0, 0, 0, 2};
const uint8_t broadcast[6] = {255, 255, 255, 255, 255, 255};
void incoming(uint8_t kind, const void *payload, size_t n) {
  uint8_t frame[122] = {kind, uint8_t(n)};
  if (n)
    std::memcpy(frame + 2, payload, n);
  HostRxControl rx{-55, micros()};
  esp_now_recv_info_t info{remote, kind == 1 ? broadcast : local, &rx};
  host_receive(&info, frame, n + 2);
}
size_t transmit(ESP_NOW_Peer &peer, const uint8_t *frame, size_t n) {
  peer.onSent(true);
  if (frame[0] == 2)
    incoming(3, frame + 2, 4);
  return n;
}
int main() {
  std::string output;
  Serial.sink = &output;
  host_transmit = transmit;
  Comms robot;
  assert(robot.begin());
  incoming(1, nullptr, 0);
  for (int i = 0; i < 8; i++) {
    host_ms += 10;
    robot.update();
  }
  assert(robot.connected());
  ThrottlePacket control;
  control.session = 1;
  control.sequence = 1;
  control.connected = control.swa = true;
  control.inverted = true;
  uint8_t bytes[ThrottlePacket::expected_packet_size()];
  control.serialize(bytes);
  incoming(6, bytes, sizeof(bytes));
  host_ms += 110;
  robot.update(); // Radio callback was fresh, but queue entry is now too old.
  assert(!robot.motor_control_fresh());
  ++control.sequence;
  control.serialize(bytes);
  incoming(6, bytes, sizeof(bytes));
  robot.update();
  assert(robot.motor_control_fresh() && robot.get_throttle_packet()->inverted);
  host_ms += 90;
  incoming(6, bytes, sizeof(bytes));
  robot.update(); // Duplicate must not extend the watchdog.
  assert(robot.motor_control_fresh());
  host_ms += 10;
  assert(!robot.motor_control_fresh()); // Motor-task guard works even without
                                        // general-loop service.
  robot.update();
  assert(!robot.get_throttle_packet()->connected);
  ++control.sequence;
  control.serialize(bytes);
  incoming(6, bytes, sizeof(bytes));
  robot.update();
  assert(robot.motor_control_fresh());
  control.swa = false;
  ++control.sequence;
  control.serialize(bytes);
  incoming(6, bytes, sizeof(bytes));
  robot.update();
  assert(!robot.motor_control_fresh());
  assert(robot.get_throttle_packet()
             ->inverted); // A timeout doesn't undo physical orientation.
  // A same-pose explicit request is acknowledged once through production Comms.
  control.orientation_request = 1;
  ++control.sequence;
  control.serialize(bytes);
  incoming(6, bytes, sizeof(bytes));
  robot.update();
  const uint32_t event = robot.orientation_feedback();
  assert(event && (event & 1));
  ++control.sequence;
  control.serialize(bytes);
  incoming(6, bytes, sizeof(bytes));
  robot.update();
  assert(robot.orientation_feedback() == event);
  control.orientation_request = 2;
  ++control.sequence;
  control.serialize(bytes);
  incoming(6, bytes, sizeof(bytes));
  robot.update();
  assert(robot.orientation_feedback() != event);
}
