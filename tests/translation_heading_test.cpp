#include "rpm_heading_tracker.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

namespace {
constexpr double k = 6.2831853071795864769 / 60;
float diff(float a, double b) { return RpmHeadingTracker::wrap(a - b); }
struct Stats { float peak = 0, final_error = 0, max_rate_change = 0; };
Stats run(bool translation, bool load, bool bursts, uint32_t step_us = 10000) {
  RpmHeadingTracker tracker;
  const uint32_t origin = UINT32_MAX - 800000;
  const auto speed = [load](int64_t t) {
    if (!load) return 2100.f;
    const float down = std::max(0.f, std::min(1.f, (t - 2000000) / 300000.f));
    const float up = std::max(0.f, std::min(1.f, (t - 4000000) / 300000.f));
    return 2100.f - 180 * down + 180 * up;
  };
  Stats stats;
  double truth = 0;
  float reference = 0, old_heading = 0, old_rate = 0;
  for (uint32_t t = 0; t <= 7000000; t += 1000) {
    if (t) truth += .5 * (speed(t) + speed(int64_t(t) - 1000)) * k * .001;
    if (t % step_us) continue;
    const uint32_t age = 70000;
    const uint32_t epoch = origin + t - age;
    float rpm = speed(int64_t(t) - age);
    if (bursts && ((t >= 1600000 && t < 1620000) || (t >= 3100000 && t < 3120000)))
      rpm += 160; // Within the RPM estimator's normal tracking band.
    tracker.set_translation(translation && t >= 1000000 && t < 5000000);
    tracker.update(origin + t, rpm, {epoch, rpm, .95f});
    const auto &s = tracker.state();
    assert(std::isfinite(s.heading_rad) && std::isfinite(s.rpm));
    if (t) assert(std::abs(diff(s.heading_rad, old_heading + old_rate * k * step_us * 1e-6)) < .001);
    if (s.translation_guard >= .999f && t > 1200000)
      stats.max_rate_change = std::max(stats.max_rate_change, std::abs(s.rpm - old_rate));
    if ((t == 2100000 || t == 2500000)) {
      const float angle = s.heading_rad;
      tracker.update(origin + t, rpm, {epoch, rpm, .95f});
      assert(tracker.state().heading_rad == angle && std::isfinite(tracker.state().rpm));
    }
    old_heading = s.heading_rad; old_rate = s.rpm;
    if (t == 800000) reference = diff(s.heading_rad, truth);
    if (t >= 1200000) stats.peak = std::max(stats.peak, std::abs(diff(s.heading_rad, truth + reference)));
    if (t == 7000000) stats.final_error = std::abs(diff(s.heading_rad, truth + reference));
  }
  return stats;
}
}
int main() {
  for (uint32_t step : {10000u, 20000u}) for (bool load : {false, true}) for (bool burst : {false, true}) {
    const auto previous = run(false, load, burst, step), motion = run(true, load, burst, step);
    std::cout << "step_ms=" << step / 1000 << " load=" << load << " bursts=" << burst << " baseline peak/end="
              << previous.peak * 57.29578 << '/' << previous.final_error * 57.29578
              << " motion peak/end=" << motion.peak * 57.29578 << '/' << motion.final_error * 57.29578
              << " max_rate_step=" << motion.max_rate_change << '\n';
    assert(motion.max_rate_change <= 1800.f * step * 1e-6f + .001f);
    if (burst) {
      assert(motion.final_error < .02f);
      const auto clean = run(true, load, false, step);
      assert(motion.peak <= clean.peak + .2f);
      assert(std::abs(motion.final_error - clean.final_error) < .001f);
      assert(motion.final_error < previous.final_error * .1f);
    }
    if (load) {
      assert(motion.peak < 1.2f); // Conservative rate limits trade peak error for smoother recovery.
      assert(motion.final_error < .02f);
    }
  }
  // A persistent speed step is a real new state, not a permanently rejected
  // spike. The centered confirmation timestamp must let the model recover it.
  RpmHeadingTracker persistent;
  for (uint32_t t = 0; t <= 5000000; t += 10000) {
    persistent.set_translation(true);
    const float rpm = t < 2070000 ? 2100 : 1940;
    persistent.update(t, rpm, {t - 70000, rpm, .95f});
  }
  assert(std::abs(persistent.state().rpm - 1940) < .1f);
  assert(std::abs(persistent.state().error_rad) < .01f);
  assert(persistent.state().status == RpmHeadingTracker::Status::Translation);
  persistent.update(5500000, 0, {});
  assert(persistent.state().status == RpmHeadingTracker::Status::Unavailable);
  const float stopped = persistent.state().heading_rad;
  persistent.update(5600000, 0, {});
  assert(persistent.state().heading_rad == stopped);
  persistent.update(5700000, 1940, {5630000, 1940, .95f});
  assert(persistent.state().heading_rad == stopped);

}
