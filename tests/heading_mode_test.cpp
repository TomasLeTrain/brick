#include "rpm_heading_tracker.h"
#include <cassert>
#include <cmath>

namespace {
constexpr float k = 6.283185307179586f / 60;
using Mode = RpmHeadingTracker::CorrectionMode;
using Status = RpmHeadingTracker::Status;
float diff(float a, float b) { return RpmHeadingTracker::wrap(a - b); }
}
int main() {
  RpmHeadingTracker tracker, dead;
  tracker.set_correction_mode(Mode::AccelerationOnly);
  const uint32_t origin = UINT32_MAX - 500000;
  float previous_angle = 0, previous_rate = 0;
  for (uint32_t t = 0; t <= 4000000; t += 10000) {
    tracker.set_translation(t >= 1000000 && t < 2000000);
    const uint32_t epoch = origin + t - 100000;
    // Deliberately contradictory observations while moving must not create debt
    // to repay after neutral. Fresh post-movement data returns to the raw RPM.
    const float measurement = t >= 1100000 && t < 2100000 ? 2400 : 1800;
    tracker.update(origin + t, 1800, {epoch, measurement, .95f});
    dead.update(origin + t, 1800, {}, false);
    const auto &s = tracker.state();
    if (t) assert(std::abs(diff(s.heading_rad, previous_angle + previous_rate * k * .01f)) < .001f);
    assert(std::abs(diff(s.heading_rad, dead.state().heading_rad)) < .001f);
    assert(std::abs(s.rpm - 1800) < .001f);
    if (t >= 1000000 && t < 2490000) assert(s.status == Status::TranslationDisabled);
    if (t >= 2490000 && t < 2590000) assert(s.status == Status::WaitingForFreshEpoch);
    if (t >= 2590000) assert(s.status == Status::Correcting);
    previous_angle = s.heading_rad; previous_rate = s.rpm;
  }
  // Runtime enable while moving waits for a fresh epoch, preserving heading.
  tracker.set_translation(true);
  tracker.set_correction_mode(Mode::Full);
  tracker.update(origin + 4010000, 1800, {origin + 3910000, 1800, .95f});
  assert(tracker.state().status == Status::WaitingForFreshEpoch);
  for (uint32_t t = 4020000; t <= 4300000; t += 10000)
    tracker.update(origin + t, 1800, {origin + t - 100000, 1800, .95f});
  assert(tracker.state().status == Status::Translation);
  // Off uses the last RAW speed, including during an RPM loss, rather than
  // coasting at an old acceleration/heading-feedback prediction.
  RpmHeadingTracker active;
  float nominal = 0;
  for (uint32_t t = 0; t < 1000000; t += 10000) {
    nominal = 1200 + t * .0009f;
    active.update(t, nominal, {t - 100000, nominal, .95f});
  }
  active.set_correction_mode(Mode::Disabled);
  active.update(1000000, 0, {});
  assert(active.state().status == Status::DeadReckoning);
  assert(active.state().rpm == nominal);
  const float angle = active.state().heading_rad;
  active.update(1010000, 0, {});
  assert(std::abs(diff(active.state().heading_rad, angle + nominal * k * .01f)) < .001f);
  active.update(1400000, 0, {});
  assert(active.state().status == Status::Unavailable);
  // Re-enabling cannot consume an old observation or restore a past phase error.
  const float stopped = active.state().heading_rad;
  active.set_correction_mode(Mode::Full);
  active.update(1410000, nominal, {1300000, nominal, .95f});
  assert(active.state().heading_rad == stopped);
  assert(active.state().status == Status::WaitingForFreshEpoch);
  active.update(1420000, nominal, {1420000, nominal, .95f});
  assert(std::abs(diff(active.state().heading_rad, stopped + nominal * k * .01f)) < .001f);
}
