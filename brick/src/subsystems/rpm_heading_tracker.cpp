#include "rpm_heading_tracker.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr float pi = 3.14159265358979323846f, tau = 2 * pi, k = tau / 60;
constexpr uint32_t coast_us = 300000, max_age_us = 250000;
constexpr float max_acceleration = 6000, max_correction_rad_s = 4;
bool valid_rpm(float rpm) { return std::isfinite(rpm) && rpm >= 350 && rpm <= 4000; }
}
float RpmHeadingTracker::wrap(float angle) {
  angle = std::fmod(angle, tau);
  if (angle > pi) angle -= tau;
  if (angle <= -pi) angle += tau;
  return angle;
}
void RpmHeadingTracker::reset() {
  head = size = 0;
  started = have_rate = anchored = false;
  heading = rate = anchor_heading = anchor_rpm = prediction_rpm = acceleration = feedback_rad_s = 0;
  output = {};
}
float RpmHeadingTracker::slope(float &fitted_rpm) const {
  fitted_rpm = anchor_rpm;
  if (size < 4) return 0;
  const auto &newest = speeds[(head + speeds.size() - 1) % speeds.size()];
  float sum_t = 0, sum_y = 0, energy = 0, cross = 0, y_energy = 0;
  unsigned count = 0;
  uint32_t span = 0;
  for (unsigned n = 0; n < size; ++n) {
    const auto &sample = speeds[(head + speeds.size() - 1 - n) % speeds.size()];
    const uint32_t age = newest.time - sample.time;
    if (age > 150000) break;
    const float t = -float(age) * 1e-6f;
    const float y = sample.rpm - newest.rpm;
    sum_t += t; sum_y += y;
    energy += t * t; cross += t * y; y_energy += y * y;
    span = age;
    ++count;
  }
  if (count < 4 || span < 40000) return 0;
  const float denominator = energy - sum_t * sum_t / count;
  if (denominator < 1e-6f) return 0;
  const float covariance = cross - sum_t * sum_y / count;
  const float variance = std::max(.001f, y_energy - sum_y * sum_y / count);
  const float raw_slope = covariance / denominator;
  const float r_squared = covariance * covariance / (denominator * variance);
  const float weight = std::max(0.f, std::min(1.f, (r_squared - .25f) / .5f));
  const bool significant = std::abs(raw_slope) * span * 1e-6f >=
                           std::max(25.f, .015f * anchor_rpm);
  const float trend = significant ? std::max(-max_acceleration,
      std::min(max_acceleration, raw_slope * weight)) : 0;
  // Predict a fitted speed, not the last noisy RPM sample. Weak trends reduce
  // to a local average instead of magnifying correlation-window noise.
  fitted_rpm = std::max(350.f, std::min(4000.f,
      newest.rpm + sum_y / count - trend * sum_t / count));
  return trend;
}
void RpmHeadingTracker::projection(uint32_t age_us, float &angle, float &rpm) const {
  const float age = age_us * 1e-6f;
  // Limit both the prediction horizon and the speed change. After this ramp,
  // extrapolate at the bounded final speed instead of accelerating forever.
  float ramp = std::min(age, .12f);
  const float limit = std::min(600.f, .2f * prediction_rpm);
  if (std::abs(acceleration) > 1e-3f) ramp = std::min(ramp, limit / std::abs(acceleration));
  const float delta = acceleration * ramp;
  rpm = prediction_rpm + delta;
  angle = wrap(anchor_heading + k * (prediction_rpm * age +
      .5f * acceleration * ramp * ramp + delta * (age - ramp)));
}
void RpmHeadingTracker::observe(uint32_t now, const Observation &m) {
  const uint32_t age = now - m.timestamp_us;
  if (!valid_rpm(m.rpm) || !std::isfinite(m.correlation) || m.correlation < .60f ||
      int32_t(age) < 0 || age > max_age_us) return;
  if (anchored && int32_t(m.timestamp_us - anchor_us) <= 0) return;
  if (!anchored || now - last_observation > coast_us || m.timestamp_us - anchor_us > coast_us) {
    // First lock / a long gap has no trustworthy past heading to repair.
    // Preserve the live heading rather than invent missed rotations.
    anchor_heading = wrap(heading - k * m.rpm * age * 1e-6f);
    acceleration = 0;
    head = size = 0;
    anchored = true;
  } else {
    const float dt = (m.timestamp_us - anchor_us) * 1e-6f;
    // Integrating *between measurement epochs* replaces the old predicted
    // history. This is the retrospective heading correction, not an RF phase.
    anchor_heading = wrap(anchor_heading + k * .5f * (anchor_rpm + m.rpm) * dt);
  }
  anchor_us = m.timestamp_us;
  anchor_rpm = m.rpm;
  last_observation = now;
  speeds[head] = {anchor_us, anchor_rpm};
  head = (head + 1) % speeds.size();
  size = std::min(size + 1, unsigned(speeds.size()));
  const float target = slope(prediction_rpm);
  acceleration += .35f * (target - acceleration);
}
void RpmHeadingTracker::update(uint32_t now, float nominal, const Observation &m, bool compensate) {
  if (!started) { started = true; last_update = now; }
  const uint32_t dt_us = now - last_update;
  float applied = 0;
  if (have_rate) {
    const uint32_t age = last_update - last_rate;
    const uint32_t usable = age < coast_us ? std::min(dt_us, coast_us - age) : 0;
    heading = wrap(heading + rate * k * usable * 1e-6f);
    applied = feedback_rad_s * usable * 1e-6f;
  }
  last_update = now;
  if (valid_rpm(nominal)) {
    last_rate = now;
    have_rate = true;
    rate = nominal;
  }
  const bool live = have_rate && now - last_rate < coast_us;
  output = {};
  output.timestamp_us = now;
  output.heading_rad = heading;
  output.correction_rad = applied;
  output.rpm = live ? rate : 0;
  output.status = live ? Status::DeadReckoning : Status::Unavailable;
  if (!live) { anchored = false; size = head = 0; acceleration = feedback_rad_s = 0; return; }
  if (!compensate) {
    anchored = false; size = head = 0; acceleration = feedback_rad_s = 0;
    return;
  }
  observe(now, m);
  if (!anchored || now - last_observation > coast_us) {
    anchored = false;
    if (!valid_rpm(nominal)) rate = std::max(0.f, rate - feedback_rad_s / k);
    feedback_rad_s = 0;
    output.rpm = rate;
    output.status = Status::Coasting;
    return;
  }
  const uint32_t age = now - anchor_us;
  float target_heading, predicted_rpm;
  projection(age, target_heading, predicted_rpm);
  output.measurement_age_ms = age / 1000.f;
  output.acceleration_rpm_s = acceleration;
  const float error = wrap(target_heading - heading);
  // Repair delayed-history error as an angular velocity, not a phase step.
  // The fast task extrapolates this total heading rate between publications.
  feedback_rad_s = std::max(-max_correction_rad_s,
                           std::min(max_correction_rad_s, error / .1f));
  rate = predicted_rpm + feedback_rad_s / k;
  output.heading_rad = heading;
  output.error_rad = error;
  output.rpm = rate;
  output.status = Status::Correcting;
}
