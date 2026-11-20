#include "../brick/src/subsystems/led_display.h"
#include "../brick/src/subsystems/melt.h"
#include <array>
#include <cassert>
#include <cmath>
#include <limits>

uint32_t host_ms = 1000;

static void expect_blue_mask(const uint8_t *red, const uint8_t *green,
                             const uint8_t *blue, uint8_t mask) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    assert(red[i] == 0 && green[i] == 0);
    assert(blue[i] == (((mask >> i) & 1) ? 255 : 0));
  }
}

int main() {
  LedDisplay display;
  std::array<uint8_t, NUM_LEDS + 2> red, green, blue;
  const auto check_column = [&](float pct, uint8_t mask) {
    red.fill(0xa5);
    green.fill(0xa5);
    blue.fill(0xa5);
    display.fast_update(pct, red.data() + 1, green.data() + 1, blue.data() + 1);
    expect_blue_mask(red.data() + 1, green.data() + 1, blue.data() + 1, mask);
    for (const auto *channel : {&red, &green, &blue})
      assert(channel->front() == 0xa5 && channel->back() == 0xa5);
  };
  check_column(0, 0); // Motor task can render before first publication.
  for (int i = 0; i < 4; ++i)
    display.desiredNums()[i] = 8;
  display.general_update();
  for (int pixel = 0; pixel < 16; ++pixel) {
    const uint8_t expected[] = {0b11111, 0b10101, 0b11111, 0};
    check_column((pixel + .5f) / 16, expected[pixel % 4]);
  }
  // Editing the next number doesn't mutate the snapshot being rendered.
  display.desiredNums()[0] = 255;
  check_column(0, 0b11111);
  display.general_update();
  check_column(0, 0);
  check_column(4.5f / 16, 0b11111); // Other digits are unaffected.
  for (float pct : {-1.f, 1.01f, std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity()})
    check_column(pct, 0);
  check_column(1, 0); // Inclusive window endpoint is the trailing space.

  // Exercise the real heading-window integration on both sides of zero.
  Melt melt;
  auto &input = *melt.get_input();
  input = {};
  input.manual_drive_type = DriveType::Melting;
  input.target_rpm_control = .6f;
  input.is_upright = true;
  input.rpm = 8888;
  melt.get_throtte_pid()->setFeedforward(.0001f);
  melt.general_update();
  constexpr float tau = 6.2831853071795864769f;
  const auto render = [&](float revolution) {
    melt.set_phase_reference(host_ms * 1000, revolution * tau, 1500);
    melt.fast_update();
    return melt.get_output();
  };
  for (int pixel = 0; pixel < 16; ++pixel) {
    const float position = std::fmod(.75f + .5f * (pixel + .5f) / 16, 1.f);
    const auto *out = render(position);
    const uint8_t expected[] = {0b11111, 0b10101, 0b11111, 0};
    expect_blue_mask(out->led_red, out->led_green, out->led_blue,
                     expected[pixel % 4]);
  }
  // Neutral translation must not punch a black hole at the window center.
  auto *out = render(0);
  expect_blue_mask(out->led_red, out->led_green, out->led_blue, 0b11111);
  out = render(.5f);
  expect_blue_mask(out->led_red, out->led_green, out->led_blue, 0);

  // A real translation marker still overrides the digits in yellow.
  input.vx = 1;
  melt.general_update();
  out = render(0);
  const int lit = int(NUM_LEDS / std::sqrt(2.f));
  for (int i = 0; i < NUM_LEDS; ++i) {
    assert(out->led_red[i] == (i < lit ? 255 : 0));
    assert(out->led_green[i] == (i < lit ? 255 : 0));
    assert(out->led_blue[i] == 0);
  }

  // Without measured spin, solid white must survive the heading rendering.
  input.rpm = 0;
  input.vx = 0;
  melt.general_update();
  for (float position : {0.f, .5f, .9f}) {
    out = render(position);
    for (int i = 0; i < NUM_LEDS; ++i)
      assert(out->led_red[i] == 255 && out->led_green[i] == 255 &&
             out->led_blue[i] == 255);
  }
}
