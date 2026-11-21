#pragma once

#include "../../config.h"
#include <cstdint>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>

// simple helper functions
uint32_t colorToInt(uint8_t red, uint8_t green, uint8_t blue);
void fill_color(uint8_t target_red, uint8_t target_green, uint8_t target_blue,
                uint8_t red[], uint8_t green[], uint8_t blue[]);
void color_first_n(uint8_t target_red, uint8_t target_green,
                   uint8_t target_blue, uint8_t red[], uint8_t green[],
                   uint8_t blue[], int color_n);

class LED {
public:
  explicit LED(gpio_num_t gpio = NEOPIXEL_PIN);
  LED(const LED &) = delete;
  LED &operator=(const LED &) = delete;
  ~LED();

  void set_brightness(uint8_t brightness) { this->brightness = brightness; }
  bool wait_idle(uint32_t timeout_ms);
  bool set_palette_flash(bool active) {
    const bool changed = palette_flash != active;
    palette_flash = active;
    return changed;
  }

  void leds_on_ready();
  void leds_on_low_battery();
  void leds_on_controller_stale();
  void leds_on_no_controller();
  void leds_on_switch_off();

  void leds_on_gradient(uint8_t color);
  void leds_off();
  void leds_on_rgb(uint8_t red, uint8_t green, uint8_t blue);

  void leds_on_array(const uint8_t red[], const uint8_t green[],
                     const uint8_t blue[], bool bottom_face = false);

private:
  void write_pixels();

  bool init_rmt(gpio_num_t gpio);

  void cleanup();

  // Per-output storage: RMT retains these symbols until transmission completes.
  rmt_symbol_word_t led_rmt_data[3 * NUM_LEDS * 8]{};
  uint8_t pixel_color[3 * NUM_LEDS]{};
  uint8_t brightness = 255;
  bool palette_flash = false;

  rmt_channel_handle_t _rmt_tx_channel = nullptr; // RMT transmit channel handle
  rmt_encoder_handle_t _enc = nullptr;
};
