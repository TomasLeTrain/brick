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
}

int main() {
  for (float rpm : {400.f, 1500.f, 2100.f, 3000.f}) {
    RssiPhaseTracker tracker;
    const uint32_t radio_origin = UINT32_MAX - 200000, local_origin = UINT32_MAX - 100000;
    float reference = 0, error_energy = 0;
    unsigned checks = 0, locks = 0;
    bool anchored = false;
    for (uint32_t t = 0; t < 10000000; t += 1000) {
      const float angle = tau * rpm * t / 60000000.0f;
      const int rssi = std::lround(-64 + 8 * std::cos(angle + .7f) + 3 * std::cos(2 * angle) + noise());
      if (t % 9000 != 0)
        tracker.add_sample(radio_origin + t, local_origin + t + 80 + t % 37, rssi);
      if (t % 10000) continue;
      const uint32_t now = local_origin + t + 6000;
      tracker.update(now, rpm * .98f);
      const auto &s = tracker.state();
      assert(std::isfinite(s.phase_rad) && std::isfinite(s.rpm));
      if (s.status == RssiPhaseTracker::Status::Tracking) {
        ++locks;
        const float truth = tau * rpm * (t + 6000) / 60000000.f;
        if (!anchored) {
          reference = RssiPhaseTracker::wrap(s.phase_rad - truth);
          anchored = true;
        }
        if (t > 2000000) {
          const float error = RssiPhaseTracker::wrap(s.phase_rad - truth - reference);
          error_energy += error * error;
          ++checks;
        }
      }
    }
    std::cout << rpm << " RPM: locks=" << locks << " RMS phase error="
              << std::sqrt(error_energy / std::max(1u, checks)) * 180 / pi << " degrees\n";
    assert(checks > 500);
    assert(std::sqrt(error_energy / checks) < .3f);
  }

  // Two identical lobes have no observable full-rotation phase. Do not lock
  // to one of the two equally plausible headings separated by 180 degrees.
  RssiPhaseTracker symmetric;
  for (uint32_t t = 0; t < 2000000; t += 1000) {
    symmetric.add_sample(t, t + 50, std::lround(-64 + 10 * std::cos(tau * 1500 * t / 30000000.f)));
    if (t % 10000 == 0) {
      symmetric.update(t + 100, 1500);
      assert(symmetric.state().status != RssiPhaseTracker::Status::Tracking);
    }
  }
  assert(symmetric.state().status == RssiPhaseTracker::Status::Ambiguous);

  RssiPhaseTracker fading;
  for (uint32_t t = 0; t < 2000000; t += 1000) {
    fading.add_sample(t, t + 50, std::lround(-80 + 20.f * t / 1000000 + noise()));
    if (t % 10000 == 0) {
      fading.update(t + 50, 2100);
      assert(fading.state().status != RssiPhaseTracker::Status::Tracking);
    }
  }

  // Lagged RPM during acceleration accumulates dead-reckoning error. Phase
  // observations must settle back to the same RF reference afterward.
  RssiPhaseTracker accelerating, dead;
  double truth = 0;
  float reference = 0, rms = 0, dead_rms = 0;
  float transient_peak = 0;
  unsigned checks = 0;
  bool have_reference = false;
  const auto speed = [](int64_t t) {
    return 1200.f + 1800.f * std::max(0.f, std::min(1.f, (t - 1000000) / 2000000.f));
  };
  for (uint32_t t = 0; t < 8000000; t += 1000) {
    truth += tau * speed(t) / 60000;
    const int rssi = std::lround(-64 + 8 * std::cos(truth + .7) +
                                 3 * std::cos(2 * truth) + noise());
    accelerating.add_sample(t, t + 50, rssi);
    if (t % 10000) continue;
    const float measured = .99f * speed(int64_t(t) - 100000);
    accelerating.update(t + 50, measured);
    dead.update(t + 50, measured, false);
    if (!have_reference && accelerating.state().status == RssiPhaseTracker::Status::Tracking) {
      reference = RssiPhaseTracker::wrap(accelerating.state().phase_rad - truth);
      have_reference = true;
    }
    if (have_reference && t >= 1000000 && t <= 3800000)
      transient_peak = std::max(transient_peak, std::abs(RssiPhaseTracker::wrap(
          accelerating.state().phase_rad - truth - reference)));
    if (t > 4000000) {
      const float error = RssiPhaseTracker::wrap(accelerating.state().phase_rad - truth - reference);
      const float dead_error = RssiPhaseTracker::wrap(dead.state().phase_rad - truth - reference);
      rms += error * error;
      dead_rms += dead_error * dead_error;
      ++checks;
    }
  }
  std::cout << "acceleration RMS phase=" << std::sqrt(rms / checks) * 180 / pi
            << " dead reckoning=" << std::sqrt(dead_rms / checks) * 180 / pi
            << " transient peak=" << transient_peak * 180 / pi << " degrees\n";
  assert(have_reference && std::sqrt(rms / checks) < .4f);
  assert(rms < dead_rms / 4);
  assert(transient_peak < 1.f);

  // A short outage coasts at the last measured rate. It must not re-zero the
  // RF reference when data resumes; complete loss eventually stops prediction.
  const float before_gap = accelerating.state().phase_rad;
  accelerating.update(8000050, 0);
  assert(std::abs(RssiPhaseTracker::wrap(accelerating.state().phase_rad - before_gap)) > .1f);
  for (uint32_t t = 8010000; t < 8100000; t += 10000) accelerating.update(t + 50, 0);
  assert(accelerating.state().status == RssiPhaseTracker::Status::Coasting);
  assert(accelerating.state().rpm > 2500);
  const double last_truth = truth;
  for (uint32_t t = 8100000; t < 9000000; t += 1000) {
    const double resumed_truth = last_truth + tau * 3000 * (t - 7999000) / 60000000.0;
    accelerating.add_sample(t, t + 50, std::lround(-64 + 8 * std::cos(resumed_truth + .7) +
                                                  3 * std::cos(2 * resumed_truth) + noise()));
    if (t % 10000 == 0) {
      accelerating.update(t + 50, 2970);
      if (t >= 8600000) {
        assert(accelerating.state().status == RssiPhaseTracker::Status::Tracking);
        assert(std::abs(RssiPhaseTracker::wrap(accelerating.state().phase_rad -
                                              resumed_truth - reference)) < .4f);
      }
    }
  }
  accelerating.update(9400050, 0);
  assert(accelerating.state().status == RssiPhaseTracker::Status::Unavailable);
  const float stopped = accelerating.state().phase_rad;
  accelerating.update(10400050, 0);
  assert(accelerating.state().phase_rad == stopped);
}
