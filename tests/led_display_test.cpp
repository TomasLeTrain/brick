#include "../brick/src/subsystems/led_display.h"
#include "../brick/src/subsystems/melt.h"
#include <array>
#include <cassert>
#include <cmath>
#include <limits>

uint32_t host_ms = 1000;
constexpr float tau = 6.2831853071795864769f;

static void expect_color(const MeltOutput &out, int pixel, LedColor color,
                         int tolerance = 0) {
  assert(std::abs(int(out.led_red[pixel]) - color.red) <= tolerance);
  assert(std::abs(int(out.led_green[pixel]) - color.green) <= tolerance);
  assert(std::abs(int(out.led_blue[pixel]) - color.blue) <= tolerance);
}
static void expect_body(const MeltOutput &out, LedColor color) {
  for (int i = 0; i < LedDisplay::data_pixels; ++i)
    expect_color(out, i, color);
}
static void expect_mask(const uint8_t *red, const uint8_t *green,
                        const uint8_t *blue, uint8_t mask) {
  for (int i = 0; i < LedDisplay::data_pixels; ++i) {
    const int bit = LedDisplay::data_pixels - 1 - i;
    const bool lit = bit >= 0 && bit < 5 && ((mask >> bit) & 1);
    const auto color = lit ? LedPalette::text : LedPalette::background;
    assert(red[i] == color.red && green[i] == color.green &&
           blue[i] == color.blue);
  }
}

int main() {
  LedDisplay display;
  std::array<uint8_t, NUM_LEDS + 2> red, green, blue;
  const auto column = [&](float pct, uint8_t mask) {
    red.fill(0xa5);
    green.fill(0xa5);
    blue.fill(0xa5);
    display.fast_update(pct, red.data() + 1, green.data() + 1, blue.data() + 1);
    expect_mask(red.data() + 1, green.data() + 1, blue.data() + 1, mask);
    assert(red[NUM_LEDS] == 255 && green[NUM_LEDS] == 255 &&
           blue[NUM_LEDS] == 255);
    for (const auto *channel : {&red, &green, &blue})
      assert(channel->front() == 0xa5 && channel->back() == 0xa5);
  };
  column(0, 0); // Unpublished digits are background, never uninitialized data.
  for (int i = 0; i < 4; ++i)
    display.desiredNums()[i] = 8;
  display.general_update();
  const uint8_t eight[] = {31, 21, 31, 0};
  for (int i = 0; i < 16; ++i)
    column((i + .5f) / 16, eight[(15 - i) % 4]);
  column(1, 31); // Horizontal sweep reversal remains from the user's commit.
  display.desiredNums()[0] = 255;
  column(15.5f / 16, 31); // Staging doesn't affect the published number.
  display.general_update();
  column(15.5f / 16, 0);
  column(11.5f / 16, 31);
  for (float pct : {-1.f, 1.01f, std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity()})
    column(pct, 0);

  Melt melt;
  auto &input = *melt.get_input();
  input = {};
  input.manual_drive_type = DriveType::Melting;
  input.target_rpm_control = .6f;
  input.is_upright = true;
  input.rpm = 8888;
  melt.get_throtte_pid()->setFeedforward(.0001f);
  melt.general_update();
  const auto render = [&](float turns) {
    melt.set_phase_reference(host_ms * 1000, turns * tau, 1500);
    melt.fast_update();
    return *melt.get_output();
  };
  // Text remains centered at pi, reversed left-to-right and rounded to 8800.
  const uint8_t zero[] = {31, 17, 31, 0};
  for (int i = 0; i < 16; ++i) {
    const float turns = .25f + .5f * (i + .5f) / 16;
    const auto out = render(turns);
    const int source = 15 - i;
    const uint8_t mask = source / 4 < 2 ? eight[source % 4] : zero[source % 4];
    expect_mask(out.led_red, out.led_green, out.led_blue,
                mask); // Ignore wheel below.
  }
  // The new 0.3-turn bar crosses zero without covering the separate text range.
  for (float turns : {0.f, .149f, .851f})
    expect_body(render(turns), LedPalette::text);
  for (float turns : {.151f, .2f, .8f, .849f})
    expect_body(render(turns), LedPalette::background);
  const LedColor wheel_colors[] = {{255, 0, 0},   {255, 255, 0}, {0, 255, 0},
                                   {0, 255, 255}, {0, 0, 255},   {255, 0, 255}};
  for (int i = 0; i < 6; ++i)
    expect_color(render(i / 6.f), LedDisplay::wheel_pixel, wheel_colors[i], 1);
  expect_color(render(1), LedDisplay::wheel_pixel, wheel_colors[0]);

  // Vector paints only its own rows over the bar; it never covers the hue
  // pixel.
  input.vx = 1;
  melt.general_update();
  auto out = render(0);
  const int count = int(LedDisplay::data_pixels / std::sqrt(2.f));
  for (int i = 0; i < LedDisplay::data_pixels; ++i)
    expect_color(out, i, i < count ? LedPalette::vector : LedPalette::text);
  expect_color(out, LedDisplay::wheel_pixel, {255, 0, 0});
  input.vx = 0;
  melt.general_update();
  const auto behind = render(.5f);
  input.vx = -1;
  melt.general_update();
  out = render(.5f);
  for (int i = 0; i < count; ++i)
    expect_color(out, i, LedPalette::vector);
  for (int i = count; i < NUM_LEDS; ++i)
    expect_color(out, i,
                 {behind.led_red[i], behind.led_green[i], behind.led_blue[i]});

  // Inversion swaps the entire palette, not the positions of the ranges.
  for (float turns : {0.f, .2f, .3f, .5f, .8f, .99f}) {
    input.is_upright = true;
    melt.general_update();
    const auto normal = render(turns);
    input.is_upright = false;
    melt.general_update();
    const auto flipped = render(turns);
    for (int i = 0; i < NUM_LEDS; ++i)
      expect_color(flipped, i,
                   LedPalette::flip({normal.led_red[i], normal.led_green[i],
                                     normal.led_blue[i]},
                                    i == LedDisplay::wheel_pixel));
  }
  input.vx = 0;
  input.is_upright = true;
  auto &config = *melt.get_config();
  config.bar_led_color = {100, 20, 70};
  melt.general_update();
  expect_body(render(0), config.bar_led_color);
  config.bar_led_width = 0;
  melt.general_update();
  expect_body(render(0), LedPalette::background);
  config.bar_led_width = 1;
  melt.general_update();
  expect_body(render(.2f), config.bar_led_color);
  config.bar_led_width = .3f;
  config.heading_led_width = 1;
  melt.general_update();
  // Text takes priority over a bar even when its range is expanded to overlap.
  const auto overlap = render(.09375f);
  expect_color(overlap, 0, LedPalette::background);

  // RPM loss/tank mode keep the background and hue pixel, without stale ranges.
  input.rpm = 0;
  melt.general_update();
  expect_body(render(0), LedPalette::background);
  expect_color(render(0), LedDisplay::wheel_pixel, {255, 0, 0});
  input.is_upright = false;
  melt.general_update();
  expect_body(render(0), LedPalette::text);
  expect_color(render(0), LedDisplay::wheel_pixel, {0, 255, 255});
  input.manual_drive_type = DriveType::Tank;
  input.is_upright = true;
  melt.general_update();
  expect_body(render(.5f), LedPalette::background);
  expect_color(render(.5f), LedDisplay::wheel_pixel, {0, 255, 255}, 1);
  // Latency shift rotates the bar and the wheel together, not just the text.
  input.manual_drive_type = DriveType::Melting;
  input.rpm = 1500;
  input.led_shift_amount = .25f;
  config.heading_led_width = .5f;
  config.bar_led_color = LedPalette::text;
  melt.general_update();
  expect_body(render(.75f), LedPalette::text);
  expect_color(render(.75f), LedDisplay::wheel_pixel, {255, 0, 0}, 1);
}
