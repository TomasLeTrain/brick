#include "led_display.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// NOTE: assumes 8 bar leds
uint8_t mask_0[4] = {0b00011111, 0b00010001, 0b00011111, 0b00000000};
uint8_t mask_1[4] = {0b00000000, 0b00000000, 0b00011111, 0b00000000};
uint8_t mask_2[4] = {0b00010111, 0b00010101, 0b00011101, 0b00000000};
uint8_t mask_3[4] = {0b00010101, 0b00010101, 0b00011111, 0b00000000};
uint8_t mask_4[4] = {0b00011100, 0b00000100, 0b00011111, 0b00000000};
uint8_t mask_5[4] = {0b00011101, 0b00010101, 0b00010111, 0b00000000};
uint8_t mask_6[4] = {0b00011111, 0b00010101, 0b00010111, 0b00000000};
uint8_t mask_7[4] = {0b00010000, 0b00010000, 0b00011111, 0b00000000};
uint8_t mask_8[4] = {0b00011111, 0b00011011, 0b00011111, 0b00000000};
uint8_t mask_9[4] = {0b00011101, 0b00010101, 0b00011111, 0b00000000};

uint8_t *digit_masks[10] = {
    mask_0, mask_1, mask_2, mask_3, mask_4,
    mask_5, mask_6, mask_7, mask_8, mask_9,
};

void LedDisplay::general_update() {
  // num ix 3 pixels wide, 5 tall
  // space is 1 pixel wide
  // so must divide width into 3 * 4 + 3 = 15 pixels
  // can simplify by having each character have the space builtin, divide into
  // 16 pixels and index pixel at given character
  // update mask
  for (int i = 0; i < 4; i++) {
    memcpy(&led_mask[i * 4], digit_masks[desired_nums[i]], 4);
  }
}

void LedDisplay::fast_update(float pct, uint8_t led_red[NUM_LEDS],
                             uint8_t led_green[NUM_LEDS],
                             uint8_t led_blue[NUM_LEDS]) {
  int32_t pixel = std::min(int(16 * pct), 15);
  uint8_t curr_mask = led_mask[pixel];

  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;

  for (int i = 0; i < 8; i++) {
    if ((curr_mask >> i) & 1) {
      led_red[i] = red;
      led_green[i] = green;
      led_blue[i] = blue;
    } else {
      led_red[i] = 0;
      led_green[i] = 0;
      led_blue[i] = 0;
    }
  }
}
