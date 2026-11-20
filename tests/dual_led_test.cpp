#include "../brick/src/subsystems/led.h"
#include "../brick/src/subsystems/led_palette.h"
#include <cassert>
#include <vector>
uint32_t host_ms = 1000;
int main() {
  LED top(GPIO_NUM_1), bottom(GPIO_NUM_4);
  assert(host_rmt_channels.size() == 2);
  auto *a = host_rmt_channels[0], *b = host_rmt_channels[1];
  assert(a->pin == 1 && b->pin == 4 && a->symbols == 48 && b->symbols == 48);
  top.leds_on_rgb(0, 0, 255);
  const auto *first = static_cast<const uint8_t *>(a->payload);
  std::vector<uint8_t> snapshot(first, first + a->size);
  bottom.leds_on_rgb(255, 0, 0);
  assert(a->payload != b->payload);
  assert(std::memcmp(snapshot.data(), a->payload, a->size) == 0);
  assert(std::memcmp(a->payload, b->payload, a->size) != 0);
  a->busy = false;
  assert(top.wait_idle(0) && !bottom.wait_idle(0));
  top.leds_on_rgb(0, 255, 0);
  assert(
      b->busy); // Top completion doesn't release the bottom's retained buffer.
  host_rmt_complete_all();
  assert(top.wait_idle(0) && bottom.wait_idle(0));
  const auto byte = [](HostRmtChannel *channel, int offset) {
    const auto *symbols =
        static_cast<const rmt_symbol_word_t *>(channel->payload);
    unsigned result = 0;
    for (int bit = 0; bit < 8; ++bit)
      result = (result << 1) | (symbols[offset * 8 + bit].duration0 == 8);
    return result;
  };
  uint8_t r[NUM_LEDS], g[NUM_LEDS], blue[NUM_LEDS];
  std::fill_n(r, NUM_LEDS, 255);
  std::fill_n(g, NUM_LEDS, 255);
  std::fill_n(blue, NUM_LEDS, 255);
  top.leds_on_array(r, g, blue);
  bottom.leds_on_array(r, g, blue, true);
  assert(byte(a, 0) == 254 && byte(a, 1) == 254 &&
         byte(a, 2) == 254); // White, GRB.
  assert(byte(b, 0) == 0 && byte(b, 1) == 0 &&
         byte(b, 2) == 254); // Blue background.
  host_rmt_complete_all();
  std::fill_n(r, NUM_LEDS, 0);
  std::fill_n(g, NUM_LEDS, 0);
  bottom.leds_on_array(r, g, blue, true);
  assert(byte(b, 0) == 254 && byte(b, 1) == 254 &&
         byte(b, 2) == 254); // White text.
  host_rmt_complete_all();
  std::fill_n(r, NUM_LEDS, 255);
  std::fill_n(g, NUM_LEDS, 0);
  std::fill_n(blue, NUM_LEDS, 128);
  bottom.leds_on_array(r, g, blue, true);
  assert(byte(b, 0) == 254 && byte(b, 1) == 0 &&
         byte(b, 2) == 127); // Mint vector.
  host_rmt_complete_all();
  // At the dedicated wheel pixel a blue hue becomes yellow, not white text.
  std::fill_n(r, NUM_LEDS, 0);
  std::fill_n(g, NUM_LEDS, 0);
  std::fill_n(blue, NUM_LEDS, 255);
  bottom.leds_on_array(r, g, blue, true);
  const int last = 3 * (NUM_LEDS - 1);
  assert(byte(b, last) == 254 && byte(b, last + 1) == 254 &&
         byte(b, last + 2) == 0);
  host_rmt_complete_all();
  // Opposite face applied to an inverted image restores the original palette.
  for (int i = 0; i < NUM_LEDS; ++i) {
    const bool wheel = i == NUM_LEDS - 1;
    const LedColor color = LedPalette::flip({r[i], g[i], blue[i]}, wheel);
    r[i] = color.red;
    g[i] = color.green;
    blue[i] = color.blue;
  }
  bottom.leds_on_array(r, g, blue, true);
  assert(byte(b, 0) == 0 && byte(b, 1) == 0 && byte(b, 2) == 254);
  assert(byte(b, last) == 0 && byte(b, last + 1) == 0 &&
         byte(b, last + 2) == 254);
  host_rmt_complete_all();
}
