#include "led_display.h"
#include <cmath>
#include <cstring>

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

uint8_t general_mask[16];

void LedDisplay::general_update() {
  // TODO: width to determine left/right bounds
  // TODO: have characters to display
  //
  // num ix 3 pixels wide, 5 tall
  // space is 1 pixel wide
  // so must divide width into 3 * 4 + 3 = 15 pixels
  // can simplify by having each character have the space builtin, divide into
  // 16 pixels and index pixel at given character

  uint8_t desired_nums[4] = {1, 2, 3, 4};

  // update mask
  for (int i = 0; i < 4; i++) {
    memcpy(&general_mask[i * 4], digit_masks[desired_nums[i]], 4);
  }
}

void LedDisplay::fast_update(uint8_t led_red[NUM_LEDS],
                             uint8_t led_green[NUM_LEDS],
                             uint8_t led_blue[NUM_LEDS]) {
  // TODO: calculate
  float pct_in_range = 0.5;
  // prevents out of bounds if exactly 1.0
  pct_in_range = fminf(pct_in_range, 0.99f);

  int pixel = int(pct_in_range * 16);

  uint8_t curr_mask = general_mask[pixel];

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
