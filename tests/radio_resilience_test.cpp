#include "../brick/src/lib/NowPair.h"
#include <cassert>
#include <vector>
uint32_t host_ms = 1000;
const uint8_t remote[6] = {2, 0, 0, 0, 0, 1}, local[6] = {2, 0, 0, 0, 0, 2};
const uint8_t wrong[6] = {2, 0, 0, 0, 0, 3},
              broadcast[6] = {255, 255, 255, 255, 255, 255};
std::vector<std::vector<uint8_t>> frames;
ESP_NOW_Peer *held = nullptr;
bool hold = false, immediate_failure = false;
int received = -1;
void incoming(uint8_t kind, const void *payload, size_t size,
              const uint8_t *source = remote) {
  uint8_t frame[122] = {kind, uint8_t(size)};
  if (size)
    std::memcpy(frame + 2, payload, size);
  HostRxControl rx{-55, micros()};
  esp_now_recv_info_t info{source, kind == 1 ? broadcast : local, &rx};
  host_receive(&info, frame, size + 2);
}
size_t transmit(ESP_NOW_Peer &peer, const uint8_t *frame, size_t length) {
  frames.emplace_back(frame, frame + length);
  if (frame[0] == 6 && immediate_failure) {
    immediate_failure = false;
    return 0;
  }
  if (frame[0] == 6 && hold) {
    held = &peer;
    return length;
  }
  peer.onSent(true);
  if (frame[0] == 2)
    incoming(3, frame + 2, 4);
  return length;
}
int main() {
  NowPair link(NowPair::Role::Slave, 6, 50, 250);
  link.expectPeer(remote);
  host_transmit = transmit;
  link.onRealtime(
      [](const uint8_t *p, size_t n, uint32_t, void *) {
        assert(n == 1);
        received = p[0];
      },
      nullptr);
  assert(link.begin());
  incoming(1, nullptr, 0, wrong);
  link.update();
  assert(link.remoteAddress() == nullptr);
  incoming(1, nullptr, 0);
  for (int i = 0; i < 8; i++) {
    host_ms += 10;
    link.update();
  }
  assert(link.connected());
  uint8_t value = 1;
  for (int i = 0; i < 40; i++) {
    value = i;
    incoming(6, &value, 1);
  }
  link.update();
  assert(received == 39); // A burst keeps the newest value, no RX backlog.
  value = 99;
  incoming(6, &value, 1, wrong);
  link.update();
  assert(received == 39);

  value = 11;
  assert(link.sendData(&value, 1));
  value = 12;
  assert(link.sendLatestData(&value, 1));
  hold = true;
  link.update();
  assert(held && frames.back()[0] == 6 && frames.back()[2] == 12);
  value = 13;
  link.sendLatestData(&value, 1);
  value = 14;
  link.sendLatestData(&value, 1);
  const auto sent = frames.size();
  link.update();
  assert(frames.size() == sent);
  host_ms += 2;
  hold = false;
  held->onSent(false);
  link.update();
  assert(frames.back()[0] == 6 && frames.back()[2] == 14);
  assert(link.txStats().failures == 1 &&
         link.txStats().max_completion_us >= 2000);
  link.update();
  assert(frames.back()[0] == 4 && frames.back()[2] == 11);

  hold = true;
  value = 20;
  link.sendLatestData(&value, 1);
  link.update();
  value = 21;
  link.sendLatestData(&value, 1);
  host_ms += 110;
  hold = false;
  held->onSent(true);
  const auto prior = frames.size();
  link.update();
  link.update();
  for (size_t i = prior; i < frames.size(); i++)
    assert(frames[i][0] != 6); // Expired control is discarded.
  immediate_failure = true;
  value = 22;
  link.sendLatestData(&value, 1);
  link.update();
  assert(link.txStats().failures == 2);
  const auto stats = link.txStats();
  assert(stats.attempts >= stats.completed + 1);
}
