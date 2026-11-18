#include "rpm_heading_tracker.h"
#include "radio_clock.h"
#include <cassert>
#include <cmath>
#include <iostream>

namespace {
constexpr float k = 6.283185307179586f / 60;
float diff(float a, float b) { return RpmHeadingTracker::wrap(a - b); }
float speed(int64_t t, bool braking) {
  const float x = std::max(0.f, std::min(1.f, (t - 1000000) / 2000000.f));
  return braking ? 3000 - 1800 * x : 1200 + 1800 * x;
}
}
int main() {
  for (bool braking : {false, true}) for (bool varying_delay : {false, true}) {
    RpmHeadingTracker tracker, dead;
    const uint32_t origin = UINT32_MAX - 800000;
    double truth = 0;
    float reference = 0, rms = 0, dead_rms = 0, peak = 0;
    unsigned checks = 0;
    float previous_heading = 0, previous_rate = 0;
    for (uint32_t t = 0; t <= 6000000; t += 1000) {
      if (t) truth += .5 * (speed(t, braking) + speed(int64_t(t) - 1000, braking)) * double(k) * .001;
      if (t % 10000) continue;
      const uint32_t delay = varying_delay ? (t < 2000000 ? 100000 : 60000) : 100000;
      const float rpm = speed(int64_t(t) - delay, braking);
      RpmHeadingTracker::Observation m{origin + t - delay, rpm, .95f};
      tracker.update(origin + t, rpm, m);
      dead.update(origin + t, rpm, m, false);
      const auto &s = tracker.state();
      assert(std::isfinite(s.heading_rad) && std::isfinite(s.rpm));
      assert(std::abs(s.correction_rad) <= .04001f);
      if (t) assert(std::abs(diff(s.heading_rad, previous_heading + previous_rate * k * .01f)) < .001f);
      previous_heading = s.heading_rad;
      previous_rate = s.rpm;
      if (t == 500000) reference = diff(s.heading_rad, truth);
      if (t >= 1000000 && t < 4000000) peak = std::max(peak, std::abs(diff(s.heading_rad, truth + reference)));
      if (t >= 4000000) {
        const float e = diff(s.heading_rad, truth + reference), d = diff(dead.state().heading_rad, truth + reference);
        rms += e * e; dead_rms += d * d; ++checks;
      }
    }
    std::cout << "braking=" << braking << " variable_delay=" << varying_delay
              << " RMS=" << std::sqrt(rms / checks) * 57.29578f
              << " DR=" << std::sqrt(dead_rms / checks) * 57.29578f
              << " peak=" << peak * 57.29578f << " degrees\n";
    assert(std::sqrt(rms / checks) < .08f);
    assert(rms < dead_rms / 10);
    // The first delayed observations cannot predict the onset of acceleration.
    // The correction must settle afterward without ever stepping heading.
    assert(peak < 1.4f);
  }
  RpmHeadingTracker tracker;
  for (uint32_t t = 0; t < 1000000; t += 10000)
    tracker.update(t, 1800, {t - 100000, 1800, .9f});
  const float before = tracker.state().heading_rad;
  // Old/duplicate/future/low-quality observations cannot rewrite the trajectory.
  tracker.update(1000000, 1800, {500000, 3500, .99f});
  assert(std::abs(diff(tracker.state().heading_rad, before + k * 1800 * .01f)) < .001f);
  tracker.update(1010000, 1800, {1020000, 3500, .99f});
  tracker.update(1020000, 1800, {1000000, 3500, .2f});
  assert(std::abs(tracker.state().acceleration_rpm_s) < 1);
  tracker.update(1500000, 0, {});
  assert(tracker.state().status == RpmHeadingTracker::Status::Unavailable);
  const float stopped = tracker.state().heading_rad;
  tracker.update(1600000, 0, {});
  assert(tracker.state().heading_rad == stopped);
  tracker.update(1700000, 2100, {1600000, 2100, .9f});
  assert(std::abs(diff(tracker.state().heading_rad, stopped)) < .001f);
  RadioClock bridge;
  assert(!bridge.available());
  bridge.observe(UINT32_MAX - 1000, UINT32_MAX - 500);
  bridge.observe(1000, 1550);
  assert(bridge.local(500) == 1000);
}
