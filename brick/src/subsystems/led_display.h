#pragma once
#include "../../config.h"
#include <cstdint>

// more advanced things to display while melting
class LedDisplay {
  // precomputes some values for the fast update
  void general_update();

  // writes the wanted values to the given array of colors
  void fast_update(uint8_t led_red[NUM_LEDS], uint8_t led_green[NUM_LEDS],
                   uint8_t led_blue[NUM_LEDS]);
};
