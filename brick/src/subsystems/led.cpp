#include "led.h"
#include "../../config.h"
#include <Arduino.h>
#include <cstring>

// TODO: make destructor
rmt_symbol_word_t led_rmt_data[3 * NUM_LEDS * 8];

uint8_t pixel_color[3 * NUM_LEDS];

bool LED::init_rmt(gpio_num_t gpio) {
  rmt_tx_channel_config_t tx_chan_config = {
      .gpio_num = gpio,               // GPIO number
      .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
      .resolution_hz =
          10 * 1000 *
          1000, // 10 MHz tick resolution, i.e., 1 ticks = 100 nanosecs
      .mem_block_symbols = 64, // memory block size, 64 * 4 = 256 Bytes
      .trans_queue_depth =
          4, // set the number of transactions that can pend in the background
  };

  if (rmt_new_tx_channel(&tx_chan_config, &_rmt_tx_channel) != ESP_OK) {
    return false;
  }

  if (rmt_enable(_rmt_tx_channel) != ESP_OK) {
    return false;
  }

  rmt_copy_encoder_config_t enc = {};
  if (rmt_new_copy_encoder(&enc, &_enc) != ESP_OK) {
    return false;
  }

  return true;
}

void LED::cleanup() {
  if (_rmt_tx_channel) {
    rmt_disable(_rmt_tx_channel);
    rmt_del_channel(_rmt_tx_channel);
    _rmt_tx_channel = nullptr;
  }
  if (_enc) {
    rmt_del_encoder(_enc);
    _enc = nullptr;
  }
}

LED::~LED() { cleanup(); }

LED::LED() {
  bool result = init_rmt(NEOPIXEL_PIN);

  // cleanup on failure
  if (!result) {
    cleanup();
    return;
  }
}

void LED::leds_on_ready() { leds_on_rgb(0, 0, 255); }

void LED::leds_on_low_battery() { leds_on_rgb(255, 0, 0); }

void LED::leds_on_controller_stale() {
  long now = millis();
  now /= 100;
  if ((now % 10) < 8) {
    leds_on_rgb(255, 0, 0);
  } else {
    leds_off();
  }
}

void LED::leds_on_no_controller() {
  long now = millis();
  now /= 100;
  if ((now % 10) < 2) {
    leds_on_rgb(255, 0, 0);
  } else {
    leds_off();
  }
}

void LED::leds_on_gradient(uint8_t color) {
  uint8_t green = (color > 50) ? 255 : (255 * int(color)) / 50;
  uint8_t red = (color < 50) ? 255 : (255 * (100 - int(color))) / 50;
  leds_on_rgb(red, green, 0);
}

void LED::leds_off() {
  // set all pixels to zero
  memset(pixel_color, 0, sizeof(uint8_t) * sizeof(pixel_color));
  write_pixel();
}

void LED::leds_on_rgb(uint8_t red, uint8_t green, uint8_t blue) {
  // scales all values by brightness:
  // new_val = (val / 255) * brightness
  // new_val = (val * brightness) / 255

  // NOTE: shifting (diving by 256) instead of diving by /255 for almost same
  // result
  int red_val = (int(red) * brightness) >> 8;
  int green_val = (int(green) * brightness) >> 8;
  int blue_val = (int(blue) * brightness) >> 8;

  // neopixels usually use GRB addressing rather than RGB
  for (size_t i = 0; i < sizeof pixel_color; i += 3) {
    pixel_color[i + 0] = green_val;
    pixel_color[i + 1] = red_val;
    pixel_color[i + 2] = blue_val;
  }
  write_pixel();
}

void LED::write_pixel() {
  int index = 0;
  for (uint8_t chan : pixel_color) {
    uint8_t value = chan;
    for (int bit = 7; bit >= 0; bit--) {
      if ((value >> bit) & 1) {
        led_rmt_data[index].level0 = 1;
        led_rmt_data[index].duration0 = 8;
        led_rmt_data[index].level1 = 0;
        led_rmt_data[index].duration1 = 4;
      } else {
        led_rmt_data[index].level0 = 1;
        led_rmt_data[index].duration0 = 4;
        led_rmt_data[index].level1 = 0;
        led_rmt_data[index].duration1 = 8;
      }
      index++;
    }
  }

  rmt_transmit_config_t txc = {};
  rmt_transmit(_rmt_tx_channel, _enc, led_rmt_data, sizeof(led_rmt_data), &txc);
}
