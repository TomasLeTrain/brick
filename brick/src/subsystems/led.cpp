// #include "led.h"
// #include "../../config.h"
// #include <Arduino.h>
// #include <cstring>
// #include <driver/rmt.h>
//
// #define NUM_LEDS 8
//
// rmt_item32_t led_data[3 * NUM_LEDS * 8];
// uint8_t pixel_color[3 * NUM_LEDS];
//
// LED::LED() {
//   rmt_config_t rmt_cfg = RMT_DEFAULT_CONFIG_TX(NEOPIXEL_PIN, NEOPIXEL_RMT);
//
//   rmt_cfg.clk_div = 8; // slow us down to 10mhz
//
//   rmt_config(&rmt_cfg);
//
//   rmt_driver_install(rmt_cfg.channel, 0, 0);
// }
//
// void LED::leds_on_ready() { leds_on_rgb(0, 0, 255); }
//
// void LED::leds_on_low_battery() { leds_on_rgb(255, 0, 0); }
//
// void LED::leds_on_controller_stale() {
//   long now = millis();
//   now /= 100;
//   if ((now % 10) < 8) {
//     leds_on_rgb(255, 0, 0);
//   } else {
//     leds_off();
//   }
// }
//
// void LED::leds_on_no_controller() {
//   long now = millis();
//   now /= 100;
//   if ((now % 10) < 2) {
//     leds_on_rgb(255, 0, 0);
//   } else {
//     leds_off();
//   }
// }
//
// void LED::leds_on_gradient(int color) {
//   int green = (color > 50) ? 255 : 255 * color / 50;
//   int red = (color < 50) ? 255 : 255 * (100 - color) / 50;
//   leds_on_rgb(red, green, 0);
// }
//
// void LED::leds_off() {
//   // set all pixels to zero
//   memset(pixel_color, 0, sizeof(uint8_t) * sizeof(pixel_color));
//   write_pixel();
// }
//
// void LED::leds_on_rgb(int red, int green, int blue) {
//   // neopixels usually use GRB addressing rather than RGB
//   for (size_t i = 0; i < sizeof pixel_color; i += 3) {
//     pixel_color[i + 0] = green;
//     pixel_color[i + 1] = red;
//     pixel_color[i + 2] = blue;
//   }
//   write_pixel();
// }
//
// void LED::write_pixel() {
//   int index = 0;
//   for (uint8_t chan : pixel_color) {
//     uint8_t value = chan;
//     for (int bit = 7; bit >= 0; bit--) {
//       if ((value >> bit) & 1) {
//         led_data[index].level0 = 1;
//         led_data[index].duration0 = 8;
//         led_data[index].level1 = 0;
//         led_data[index].duration1 = 4;
//       } else {
//         led_data[index].level0 = 1;
//         led_data[index].duration0 = 4;
//         led_data[index].level1 = 0;
//         led_data[index].duration1 = 8;
//       }
//       index++;
//     }
//   }
//
//   rmt_write_items(NEOPIXEL_RMT, led_data, sizeof led_data, false);
// }
