#include "led_display.h"
#include <algorithm>
#include <cmath>
namespace {
// Three columns by five rows, followed by one blank spacing column.
constexpr uint8_t digit_masks[10][4] = {
    {0b11111, 0b10001, 0b11111, 0}, {0b00000, 0b00000, 0b11111, 0},
    {0b10111, 0b10101, 0b11101, 0}, {0b10101, 0b10101, 0b11111, 0},
    {0b11100, 0b00100, 0b11111, 0}, {0b11101, 0b10101, 0b10111, 0},
    {0b11111, 0b10101, 0b10111, 0}, {0b10000, 0b10000, 0b11111, 0},
    {0b11111, 0b10101, 0b11111, 0}, {0b11101, 0b10101, 0b11111, 0},
};
} // namespace

void LedDisplay::general_update() {
  uint32_t digits = 0;
  for (int i = 0; i < 4; ++i) {
    const uint32_t digit = desired_nums[i] < 10 ? desired_nums[i] : 15;
    digits |= digit << (4 * i);
  }
  published_digits.store(digits, std::memory_order_relaxed);
}

void LedDisplay::fast_update(float pct, uint8_t led_red[NUM_LEDS],
                             uint8_t led_green[NUM_LEDS],
                             uint8_t led_blue[NUM_LEDS]) {
  uint8_t mask = 0;
  if (std::isfinite(pct) && pct >= 0 && pct <= 1) {
    int pixel = std::min(int(16 * pct), 15);
    pixel = 15 - pixel;

    const uint32_t digits = published_digits.load(std::memory_order_relaxed);
    const uint8_t digit = (digits >> (4 * (pixel / 4))) & 15;
    if (digit < 10)
      mask = digit_masks[digit][pixel % 4];
  }
  for (int i = 0; i < NUM_LEDS; ++i) {
    // Keep the merged row reversal, but move the five glyph rows off the
    // dedicated final pixel. Padding and unlit strokes use the background.
    const int bit = data_pixels - 1 - i;
    const bool lit = bit >= 0 && bit < 5 && ((mask >> bit) & 1);
    const LedColor color = lit ? LedPalette::text : LedPalette::background;
    led_red[i] = color.red;
    led_green[i] = color.green;
    led_blue[i] = color.blue;
  }
}
