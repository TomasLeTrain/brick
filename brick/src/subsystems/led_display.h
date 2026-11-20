#pragma once
#include "../../config.h"
#include <cstdint>

// more advanced things to display while melting
class LedDisplay {
public:
  // precomputes some values for the fast update
  void general_update();

  // get pointer to desired nums to write to
  uint8_t *desiredNums() { return desired_nums; }

  // writes the wanted values to the given array of colors
  void fast_update(float pct, uint8_t led_red[NUM_LEDS],
                   uint8_t led_green[NUM_LEDS], uint8_t led_blue[NUM_LEDS]);

private:
  uint8_t led_mask[16];

  uint8_t desired_nums[4] = {1, 2, 3, 4};
};
