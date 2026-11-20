#pragma once
#include "../../config.h"
#include "led_palette.h"
#include <atomic>
#include <cstdint>

// more advanced things to display while melting
class LedDisplay {
public:
  static_assert(NUM_LEDS >= 6,
                "Five glyph rows plus one hue pixel are required");
  static constexpr int wheel_pixel = NUM_LEDS - 1;
  static constexpr int data_pixels = NUM_LEDS - 1;
  // precomputes some values for the fast update
  void general_update();

  // General-task staging only; general_update() publishes to the motor task.
  uint8_t *desiredNums() { return desired_nums; }

  // writes the wanted values to the given array of colors
  void fast_update(float pct, uint8_t led_red[NUM_LEDS],
                   uint8_t led_green[NUM_LEDS], uint8_t led_blue[NUM_LEDS]);

private:
  // Publish all four digits together: the motor task must never wait for the
  // general task or see a partially updated number. Nibbles >= 10 are blank.
  std::atomic<uint32_t> published_digits{0xffff};

  uint8_t desired_nums[4] = {1, 2, 3, 4};
};
