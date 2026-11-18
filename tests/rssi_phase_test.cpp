#include "rssi_phase_tracker.h"
#include <cassert>
#include <cmath>
#include <iostream>

namespace {
constexpr float pi = 3.14159265358979323846f, tau = 2 * pi;
uint32_t random_state = 12345;
int noise() {
  random_state = random_state * 1664525u + 1013904223u;
  return int((random_state >> 16) % 7) - 3;
}
float difference(float a, float b) { return RssiPhaseTracker::wrap(a - b); }
}

int main() {
  // At steady speed, arbitrary RF phase/profile changes must not move heading.
  // This deliberately includes biased RPM: the contract is DR at steady speed,
  // rather than continuously treating transmitter bearing as absolute heading.
  for (float rpm : {400.f, 1500.f, 2100.f, 3000.f}) {
    RssiPhaseTracker tracker, dead;
    const uint32_t radio_origin = UINT32_MAX - 200000, local_origin = UINT32_MAX - 100000;
    unsigned monitoring = 0;
    for (uint32_t t = 0; t < 6000000; t += 1000) {
      const float angle = tau * rpm * t / 60000000.0f;
      const float bearing = t < 2000000 ? .7f : t < 3500000 ? 2.9f : .7f + (t - 3500000) * 1e-6f;
      const int rssi = std::lround(-64 + 8 * std::cos(angle + bearing) +
                                  3 * std::cos(2 * angle - bearing) + noise());
      if (t % 9000 != 0 && !(t > 4000000 && t < 4100000))
        tracker.add_sample(radio_origin + t, local_origin + t + 80 + t % 37, rssi);
      if (t % 10000) continue;
      const uint32_t now = local_origin + t + 6000;
      tracker.set_drive_hint(.3f + (t % 20000 ? .0005f : 0));
      tracker.update(now, rpm * .98f);
      dead.update(now, rpm * .98f, false);
      const auto &s = tracker.state();
      assert(std::isfinite(s.phase_rad) && std::isfinite(s.rpm));
      assert(s.phase_rad == dead.state().phase_rad && s.rpm == rpm * .98f);
      if (s.status == RssiPhaseTracker::Status::Monitoring) ++monitoring;
      assert(s.status != RssiPhaseTracker::Status::Tracking);
    }
    assert(monitoring > 400);
  }

  // Equal lobes and baseline fade/noise are not usable full-turn observations.
  RssiPhaseTracker symmetric, fading;
  for (uint32_t t = 0; t < 2000000; t += 1000) {
    symmetric.add_sample(t, t + 50, std::lround(-64 + 10 * std::cos(tau * 1500 * t / 30000000.f)));
    fading.add_sample(t, t + 50, std::lround(-80 + 20.f * t / 1000000 + noise()));
    if (t % 10000 == 0) {
      symmetric.set_drive_hint(t < 1000000 ? .2f : .3f);
      fading.set_drive_hint(t < 1000000 ? .2f : .3f);
      symmetric.update(t + 50, 1500);
      fading.update(t + 50, 2100);
      assert(symmetric.state().status != RssiPhaseTracker::Status::Tracking);
      assert(fading.state().status != RssiPhaseTracker::Status::Tracking);
    }
  }
  assert(symmetric.state().status == RssiPhaseTracker::Status::Ambiguous);

  // Genuine speed transitions, in either direction, with a 100 ms RPM delay.
  // Without a throttle cue, RPM changes alone must remain dead reckoning.
  for (bool braking : {false, true}) for (bool cue : {false, true}) {
    RssiPhaseTracker accelerating, dead;
    double truth = 0;
    float reference = 0, rms = 0, dead_rms = 0, peak = 0;
    unsigned checks = 0, locks = 0;
    const auto speed = [braking](int64_t t) {
      const float x = std::max(0.f, std::min(1.f, (t - 1000000) / 2000000.f));
      return braking ? 3000.f - 1800.f * x : 1200.f + 1800.f * x;
    };
    for (uint32_t t = 0; t < 6000000; t += 1000) {
      truth += tau * speed(t) / 60000;
      const int rssi = std::lround(-64 + 8 * std::cos(truth + .7) +
                                   3 * std::cos(2 * truth) + noise());
      accelerating.add_sample(t, t + 50, rssi);
      if (t % 10000) continue;
      if (cue) accelerating.set_drive_hint(speed(t < 1000000 ? 0 : 3000000) / 6784.f);
      const float measured = speed(int64_t(t) - 100000);
      accelerating.update(t + 50, measured);
      dead.update(t + 50, measured, false);
      if (t == 500000) reference = difference(accelerating.state().phase_rad, truth);
      if (accelerating.state().status == RssiPhaseTracker::Status::Tracking) ++locks;
      if (t >= 1000000 && t <= 3800000)
        peak = std::max(peak, std::abs(difference(accelerating.state().phase_rad, truth + reference)));
      if (t > 4000000) {
        const float error = difference(accelerating.state().phase_rad, truth + reference);
        const float dead_error = difference(dead.state().phase_rad, truth + reference);
        rms += error * error;
        dead_rms += dead_error * dead_error;
        ++checks;
        assert(accelerating.state().rpm == measured); // No steady integrator windup.
        assert(accelerating.state().status != RssiPhaseTracker::Status::Tracking);
      }
    }
    std::cout << "braking=" << braking << " cue=" << cue << " locks=" << locks
              << " settled RMS=" << std::sqrt(rms / checks) * 180 / pi
              << " DR=" << std::sqrt(dead_rms / checks) * 180 / pi
              << " transient peak=" << peak * 180 / pi << " degrees\n";
    if (cue) {
      assert(locks > 50 && std::sqrt(rms / checks) < .15f);
      assert(peak < .8f);
      assert(rms < dead_rms / 4);
    } else {
      assert(locks == 0 && std::abs(rms - dead_rms) < .001f);
    }
  }

  // Translation veto includes throttle/speed changes and a 500 ms neutral hold.
  // Releasing the sticks must not trigger a catch-up to the changed RF bearing.
  RssiPhaseTracker moving, dead;
  for (uint32_t t = 0; t < 4000000; t += 1000) {
    const float bearing = t < 1000000 ? 0 : 2.f;
    moving.add_sample(t, t + 50, std::lround(-64 + 8 * std::cos(tau * 2100 * t / 60000000.f + bearing)));
    if (t % 10000) continue;
    const bool translating = t >= 1000000 && t < 2000000;
    const float rpm = t < 1500000 ? 2100 : 2200;
    moving.set_drive_hint(t < 1500000 ? .3f : .4f, translating);
    moving.update(t + 50, rpm);
    dead.update(t + 50, rpm, false);
    assert(moving.state().phase_rad == dead.state().phase_rad);
    assert(moving.state().rpm == rpm);
    if (t >= 1000000 && t < 2490000)
      assert(moving.state().status == RssiPhaseTracker::Status::MotionHold);
  }

  // Persistent high-quality RF jumps during an assistance window are rejected;
  // three overlapping observations must not authorize a large heading change.
  RssiPhaseTracker jumped, jump_dead;
  float last_phase = 0, maximum_correction = 0, maximum_deviation = 0;
  unsigned rejections = 0;
  for (uint32_t t = 0; t < 3000000; t += 1000) {
    const float bearing = t < 1200000 ? .7f : 2.8f;
    jumped.add_sample(t, t + 50, std::lround(-64 + 10 * std::cos(tau * 2100 * t / 60000000.f + bearing)));
    if (t % 10000) continue;
    jumped.set_drive_hint(t < 1100000 ? .3f : .4f);
    jumped.update(t + 50, 2100);
    jump_dead.update(t + 50, 2100, false);
    maximum_deviation = std::max(maximum_deviation,
        std::abs(difference(jumped.state().phase_rad, jump_dead.state().phase_rad)));
    if (t) maximum_correction = std::max(maximum_correction,
        std::abs(difference(jumped.state().phase_rad, last_phase + tau * 2100 / 6000.f)));
    if (jumped.state().status == RssiPhaseTracker::Status::Rejected) ++rejections;
    last_phase = jumped.state().phase_rad;
  }
  assert(rejections > 10);
  assert(maximum_correction < .08f);
  assert(maximum_deviation < .15f);

  // Complete RPM loss expires instead of extrapolating indefinitely.
  jumped.update(3400050, 0);
  assert(jumped.state().status == RssiPhaseTracker::Status::Unavailable);
  const float stopped = jumped.state().phase_rad;
  jumped.update(4400050, 0);
  assert(jumped.state().phase_rad == stopped);
}
