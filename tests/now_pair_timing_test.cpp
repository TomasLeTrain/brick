#include "../brick/src/lib/NowPair.h"
#include <cassert>

uint32_t host_ms = 1000;
const uint8_t remote[6] = {2, 0, 0, 0, 0, 1}, local[6] = {2, 0, 0, 0, 0, 2};
const uint8_t broadcast[6] = {255, 255, 255, 255, 255, 255};
unsigned timed = 0, legacy = 0;
uint32_t observed_radio = 0, observed_callback = 0;

void incoming(uint8_t kind, const void *payload, size_t size, uint32_t radio_us) {
  uint8_t frame[122] = {kind, static_cast<uint8_t>(size)};
  if (size) std::memcpy(frame + 2, payload, size);
  HostRxControl rx{-55, radio_us};
  esp_now_recv_info_t info{remote, kind == 1 ? broadcast : local, &rx};
  host_receive(&info, frame, size + 2);
}

size_t outgoing(ESP_NOW_Peer &peer, const uint8_t *frame, size_t length) {
  peer.onSent(true);
  if (frame[0] == 2) incoming(3, frame + 2, 4, micros());
  return length;
}

int main() {
  NowPair link(NowPair::Role::Slave, 6, 50, 250);
  host_transmit = outgoing;
  assert(link.begin());
  incoming(1, nullptr, 0, micros());
  for (int i = 0; i < 5; ++i) { host_ms += 10; link.update(); }
  assert(link.connected());
  link.onRssi([](int, uint32_t radio_us, void *) {
    if (radio_us == 12345 || radio_us == 13345) ++legacy;
  }, nullptr);
  link.onRssiTimed([](int, uint32_t radio_us, uint32_t callback_us, void *) {
    if (radio_us != 12345 && radio_us != 13345) return;
    ++timed;
    observed_radio = radio_us;
    observed_callback = callback_us;
  }, nullptr);
  const uint32_t callback_time = micros();
  incoming(5, nullptr, 0, 12345);
  host_ms += 30; // The application task does not drain its queue immediately.
  link.update();
  assert(timed == 1 && legacy == 1);
  assert(observed_radio == 12345 && observed_callback == callback_time);
  assert(observed_callback != micros());
  link.onRssi(nullptr, nullptr);
  incoming(5, nullptr, 0, 13345);
  link.update();
  assert(timed == 2 && legacy == 1); // Removing the legacy hook must not disable collection.
}
