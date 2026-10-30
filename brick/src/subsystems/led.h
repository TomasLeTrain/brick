#pragma once

#include <driver/gpio.h>
#include <driver/rmt_tx.h>

class LED {
public:
  LED();
  ~LED();

  void leds_on_ready();
  void leds_on_low_battery();
  void leds_on_controller_stale();
  void leds_on_no_controller();

  void leds_on_gradient(int color);
  void leds_off();
  void leds_on_rgb(int red, int green, int blue);

private:
  void write_pixel();

  bool init_rmt(gpio_num_t gpio);

  void cleanup();

  rmt_channel_handle_t _rmt_tx_channel = nullptr; // RMT transmit channel handle
  rmt_encoder_handle_t _enc = nullptr;
};
