#include "../brick/src/subsystems/led.h"
#include <cassert>
#include <vector>

uint32_t host_ms = 1000;

int main() {
  LED led;
  led.set_brightness(255);
  led.leds_on_rgb(255, 0, 0);
  assert(host_rmt_transmits == 1);
  const auto *payload = static_cast<const uint8_t *>(host_rmt_payload);
  const std::vector<uint8_t> original(payload, payload + host_rmt_size);
  assert(!led.wait_idle(20));
  led.leds_on_rgb(0, 255, 0);
  assert(host_rmt_transmits == 1);
  assert(std::memcmp(original.data(), host_rmt_payload, host_rmt_size) == 0);
  host_rmt_busy = false;
  assert(led.wait_idle(20));
  led.leds_on_rgb(0, 255, 0);
  assert(host_rmt_transmits == 2);
  assert(std::memcmp(original.data(), host_rmt_payload, host_rmt_size) != 0);
  host_rmt_busy = false;
}
