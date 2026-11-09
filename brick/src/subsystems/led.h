#pragma once

#include <cstdint>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>

uint32_t colorToInt(uint8_t red, uint8_t green, uint8_t blue);

class LED {
public:
  LED();
  ~LED();

  void set_brightness(uint8_t brightness) { this->brightness = brightness; }

  void leds_on_ready();
  void leds_on_low_battery();
  void leds_on_controller_stale();
  void leds_on_no_controller();

  void leds_on_gradient(uint8_t color);
  void leds_off();
  void leds_on_rgb(uint8_t red, uint8_t green, uint8_t blue);

private:
  void write_pixel();

  bool init_rmt(gpio_num_t gpio);

  void cleanup();

  uint8_t brightness = 255;

  rmt_channel_handle_t _rmt_tx_channel = nullptr; // RMT transmit channel handle
  rmt_encoder_handle_t _enc = nullptr;
};
