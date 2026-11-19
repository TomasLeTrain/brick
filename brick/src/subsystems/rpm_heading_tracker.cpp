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
  heading = rate = anchor_heading = anchor_rpm = prediction_rpm = acceleration = feedback_rad_s = applied_feedback_rad_s = 0;
  source_seen = translation_requested = translation_seen = motion_noise_seen = false;
  motion_head = motion_size = 0;
  translation_blend = nominal_rate = 0;
  mode_changed = correction_suppressed = awaiting_epoch = false;
  output = {};
}
void RpmHeadingTracker::discard_correction() {
  anchored = false;
  head = size = motion_head = motion_size = 0;
  source_seen = motion_noise_seen = false;
  acceleration = feedback_rad_s = applied_feedback_rad_s = 0;
}
void RpmHeadingTracker::update_translation(uint32_t now, uint32_t dt_us) {
  if (motion_noise_seen && now - last_motion_noise >= 500000) motion_noise_seen = false;
  if (translation_requested) { translation_seen = true; last_translation = now; }
  if (translation_seen && !translation_requested && now - last_translation >= 500000)
    translation_seen = false;
  const float target = translation_seen ? 1.f : 0.f;
  const float step = std::min(dt_us, uint32_t(50000)) * 1e-6f /
                     (target > translation_blend ? .1f : .3f);
  translation_blend += std::max(-step, std::min(step, target - translation_blend));
}
float RpmHeadingTracker::guard_weight() const {
  // Preserve prompt tracking of coherent load-induced acceleration. Weak trends
  // and recently suspect measurements use the conservative translation gains.
  const float quiet = std::max(0.f, 1.f - std::abs(acceleration) / 300.f);
  const float noise = motion_noise_seen ? std::max(0.f, std::min(1.f,
      (500000.f - (last_update - last_motion_noise)) / 200000.f)) : 0;
  return translation_blend * std::max(noise, quiet);
}
RpmHeadingTracker::Observation RpmHeadingTracker::motion_observation(uint32_t now, const Observation &m) {
  const uint32_t age = now - m.timestamp_us;
  if (!valid_rpm(m.rpm) || !std::isfinite(m.correlation) || m.correlation < .6f ||
      int32_t(age) < 0 || age > max_age_us) return {};
  if (source_seen && int32_t(m.timestamp_us - source_time) < 0) return {};
  const bool fresh = !source_seen || int32_t(m.timestamp_us - source_time) > 0;
  if (fresh) {
    if (source_seen && m.timestamp_us - source_time > coast_us) motion_head = motion_size = 0;
    source_seen = true;
    source_time = m.timestamp_us;
    motion_samples[motion_head] = m;
    motion_head = (motion_head + 1) % motion_samples.size();
    motion_size = std::min(motion_size + 1, unsigned(motion_samples.size()));
  }
  if (translation_blend <= 0) return m;
  if (motion_size < motion_samples.size()) return m;
  // Ordinary smooth load changes do not need the median's admission delay.
  // Only suspicious innovations are held back for independent later samples.
  if (anchored && int32_t(m.timestamp_us - anchor_us) > 0) {
    const float dt = (m.timestamp_us - anchor_us) * 1e-6f;
    const float trend = std::max(-1800.f, std::min(1800.f, acceleration));
    const float expected = anchor_rpm + trend * dt;
    const float tolerance = std::max(25.f, .02f * anchor_rpm) + 1800.f * std::min(dt, .05f);
    if (std::abs(m.rpm - expected) <= tolerance) return m;
    if (fresh) { motion_noise_seen = true; last_motion_noise = now; }
  }
  std::array<float, 5> values;
  float confidence = 1;
  for (unsigned i = 0; i < values.size(); ++i) {
    values[i] = motion_samples[i].rpm;
    confidence = std::min(confidence, motion_samples[i].correlation);
  }
  std::sort(values.begin(), values.end());
  // A symmetric median describes the CENTER time, not the newest packet. Its
  // added admission delay is therefore compensated by the existing clock model.
  const auto &center = motion_samples[(motion_head + 2) % motion_samples.size()];
  return {center.timestamp_us, values[2], confidence};
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
  const float guard = guard_weight();
  const float horizon = .12f + guard * (translation_tuning.prediction_s - .12f);
  const float max_delta = 600.f + guard * (180.f - 600.f);
  const float max_ratio = .2f + guard * (.08f - .2f);
  float ramp = std::min(age, horizon);
  const float limit = std::min(max_delta, max_ratio * prediction_rpm);
  const float acceleration_limit = max_acceleration + guard * (1800.f - max_acceleration);
  const float predicted_acceleration = std::max(-acceleration_limit, std::min(acceleration_limit, acceleration));
  // Bound the increment using the acceleration actually used for projection.
  if (std::abs(predicted_acceleration) > 1e-3f)
    ramp = std::min(ramp, limit / std::abs(predicted_acceleration));
  const float delta = predicted_acceleration * ramp;
  rpm = prediction_rpm + delta;
  angle = wrap(anchor_heading + k * (prediction_rpm * age +
      .5f * predicted_acceleration * ramp * ramp + delta * (age - ramp)));
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
  const float previous_rate = rate;
  const bool previous_live = have_rate && now - last_rate < coast_us;
  update_translation(now, dt_us);
  float applied = 0;
  if (have_rate) {
    const uint32_t age = last_update - last_rate;
    const uint32_t usable = age < coast_us ? std::min(dt_us, coast_us - age) : 0;
    heading = wrap(heading + rate * k * usable * 1e-6f);
    applied = applied_feedback_rad_s * usable * 1e-6f;
  }
  last_update = now;
  if (valid_rpm(nominal)) {
    last_rate = now;
    have_rate = true;
    nominal_rate = rate = nominal;
  }
  const bool live = have_rate && now - last_rate < coast_us;
  output = {};
  output.timestamp_us = now;
  output.translation_blend = translation_blend;
  output.heading_rad = heading;
  output.correction_rad = applied;
  output.rpm = live ? rate : 0;
  output.status = live ? Status::DeadReckoning : Status::Unavailable;
  if (!live) { anchored = false; size = head = 0; acceleration = feedback_rad_s = applied_feedback_rad_s = 0; return; }
  const bool motion_disabled = correction_mode_value == CorrectionMode::AccelerationOnly && translation_seen;
  const bool suppressed = !compensate || correction_mode_value == CorrectionMode::Disabled || motion_disabled;
  if (mode_changed || suppressed != correction_suppressed) {
    discard_correction();
    resume_epoch = now;
    awaiting_epoch = true;
    mode_changed = false;
  }
  correction_suppressed = suppressed;
  if (suppressed) {
    // Never bank movement error to repay after the driver has corrected it.
    discard_correction();
    resume_epoch = now;
    awaiting_epoch = true;
    translation_blend = 0;
    rate = nominal_rate; // Includes proper raw-RPM coasting during a speed loss.
    output.rpm = rate;
    output.translation_blend = 0;
    output.status = motion_disabled ? Status::TranslationDisabled : Status::DeadReckoning;
    return;
  }
  if (awaiting_epoch) {
    // A cached measurement can arrive after neutral yet still describe movement.
    // Require an effective epoch after the suppression/settling interval.
    const uint32_t age = now - m.timestamp_us;
    if (!valid_rpm(m.rpm) || !std::isfinite(m.correlation) || m.correlation < .6f ||
        int32_t(m.timestamp_us - resume_epoch) < 0 || int32_t(age) < 0 || age > max_age_us) {
      rate = nominal_rate;
      output.rpm = rate;
      output.status = Status::WaitingForFreshEpoch;
      return;
    }
    awaiting_epoch = false;
  }
  observe(now, motion_observation(now, m));
  if (!anchored || now - last_observation > coast_us) {
    anchored = false;
    if (!valid_rpm(nominal)) rate = std::max(0.f, rate - applied_feedback_rad_s / k);
    feedback_rad_s = applied_feedback_rad_s = 0;
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
  const float guard = guard_weight();
  output.translation_guard = guard;
  const float response = .1f + guard * (translation_tuning.response_s - .1f);
  const float correction_limit = max_correction_rad_s + guard *
      (translation_tuning.max_correction_rad_s - max_correction_rad_s);
  const float target_feedback = std::max(-correction_limit, std::min(correction_limit, error / response));
  if (guard > 0) {
    const float dt = std::min(dt_us, uint32_t(50000)) * 1e-6f;
    const float slew = 4.f * dt / guard;
    feedback_rad_s += std::max(-slew, std::min(slew, target_feedback - feedback_rad_s));
    feedback_rad_s = std::max(-correction_limit, std::min(correction_limit, feedback_rad_s));
    // Do not let a noisy new slope or an error sign change jerk the heading rate.
    const float desired_rate = predicted_rpm + feedback_rad_s / k;
    // A sustained RPM trend needs prompt velocity tracking; smoothing a real
    // slowdown would reintroduce the very delay the observer compensates for.
    const float quiet = std::max(0.f, 1.f - std::abs(acceleration) / 300.f);
    const float alpha = dt > 0 ? dt / (.06f * guard * quiet + dt) : 0;
    const float change = (desired_rate - previous_rate) * alpha;
    const float limit = 1800.f * dt / guard;
    const float actual_change = std::max(-limit, std::min(limit, change));
    const float admitted = previous_live ? alpha * (std::abs(change) > limit ? limit / std::abs(change) : 1.f) : 1.f;
    applied_feedback_rad_s += admitted * (feedback_rad_s - applied_feedback_rad_s);
    rate = previous_live ? previous_rate + actual_change : desired_rate;
  } else {
    feedback_rad_s = applied_feedback_rad_s = target_feedback;
    rate = predicted_rpm + feedback_rad_s / k;
  }
  output.heading_rad = heading;
  output.error_rad = error;
  output.rpm = rate;
  output.status = translation_blend <= 0 ? Status::Correcting :
      guard > .5f ? Status::Translation : Status::TranslationTrend;
}
