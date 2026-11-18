#include "rssi_phase_tracker.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr float pi = 3.14159265358979323846f, tau = 2 * pi;
constexpr float rpm_to_rad_s = tau / 60;
constexpr uint32_t coast_us = 300000;
}

float RssiPhaseTracker::wrap(float angle) {
  angle = std::fmod(angle, tau);
  if (angle > pi) angle -= tau;
  if (angle <= -pi) angle += tau;
  return angle;
}

void RssiPhaseTracker::reset() {
  head = size = 0;
  clock_seen = started = have_rate = anchored = false;
  observation_seen = hint_seen = hint_initialized = translating = false;
  motion_seen = assisting = candidate_seen = rejected = false;
  phase = nominal_rpm = trim_rad_s = reference = 0;
  filtered_rpm = settling_rpm = 0;
  good_fits = 0;
  output = {};
}

void RssiPhaseTracker::set_drive_hint(float throttle, bool moving) {
  if (!std::isfinite(throttle) || throttle < 0 || throttle > 1) return;
  throttle_hint = throttle;
  translating = moving;
  hint_seen = true;
}

void RssiPhaseTracker::add_sample(uint32_t radio_us, uint32_t callback_us, int rssi) {
  if (rssi < -128 || rssi > 0) return;
  if (clock_seen) {
    const uint32_t elapsed = radio_us - last_radio_us;
    if (!elapsed) return;
    if (elapsed >= 0x80000000u) {
      // A genuine radio-clock restart can be remapped without discarding the
      // RF phase reference. Ordinary duplicate/out-of-order frames are ignored.
      if (last_radio_us - radio_us < 1000000 || callback_us - last_callback_us < 100000)
        return;
      clock_seen = false;
      size = head = 0;
    } else if (elapsed >= coast_us) {
      size = head = 0;
    }
  }
  const uint32_t offset = callback_us - radio_us;
  if (!clock_seen) {
    clock_offset_us = window_offset_us = offset;
    clock_window_us = callback_us;
    clock_seen = true;
  }
  // Callback latency is nonnegative. A rolling minimum maps the hardware
  // radio clock into micros(), without including the main task's queue delay.
  if (int32_t(offset - clock_offset_us) < 0) clock_offset_us = offset;
  if (int32_t(offset - window_offset_us) < 0) window_offset_us = offset;
  if (callback_us - clock_window_us >= 1000000) {
    clock_offset_us = window_offset_us;
    window_offset_us = offset;
    clock_window_us = callback_us;
  }
  last_radio_us = radio_us;
  last_callback_us = callback_us;
  if (int32_t(callback_us - (radio_us + clock_offset_us)) > 20000) return;
  if (size && radio_us - samples[(head + samples.size() - 1) % samples.size()].radio_us < 750)
    return; // Bound history/work to <=1.34 kHz, without interpolating gaps.
  samples[head] = {radio_us, float(rssi + 64)};
  head = (head + 1) % samples.size();
  size = std::min(size + 1, samples.size());
}

bool RssiPhaseTracker::fit(float rpm, uint32_t now_us, float &signal_phase,
                          uint32_t &center_us, float &confidence, bool &ambiguous) const {
  if (!size) return false;
  const uint32_t span_us = uint32_t(std::max(40000.f, std::min(300000.f, 120000000.f / rpm)));
  const uint32_t newest = samples[(head + samples.size() - 1) % samples.size()].radio_us;
  const uint32_t end_us = newest + clock_offset_us;
  if (int32_t(now_us - end_us) < 0 || now_us - end_us > 20000) return false;
  center_us = end_us - span_us / 2;
  // Constant + baseline slope + fundamental + second harmonic. Separating the
  // second harmonic prevents two antenna lobes from masquerading as full phase.
  float matrix[6][7]{}, sum = 0, energy = 0;
  unsigned count = 0;
  uint32_t oldest_age = 0, previous_age = 0;
  for (size_t n = 0; n < size; ++n) {
    const auto &sample = samples[(head + samples.size() - 1 - n) % samples.size()];
    const uint32_t age = newest - sample.radio_us;
    if (age > span_us) break;
    if (n && age - previous_age > 8000) return false;
    previous_age = oldest_age = age;
    const float u = 1.f - 2.f * float(age) / span_us;
    const float angle = (float(span_us / 2) - age) * 1e-6f * rpm * rpm_to_rad_s;
    const float c = std::cos(angle), s = std::sin(angle);
    const float basis[6] = {1, u, c, s, 2 * c * c - 1, 2 * c * s};
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) matrix[i][j] += basis[i] * basis[j];
      matrix[i][6] += basis[i] * sample.rssi;
    }
    sum += sample.rssi;
    energy += sample.rssi * sample.rssi;
    ++count;
  }
  if (count < 16 || count < span_us / 2500 || oldest_age < span_us * 9 / 10) return false;
  float rhs[6];
  for (int i = 0; i < 6; ++i) rhs[i] = matrix[i][6];
  const float time_energy = matrix[1][1] - matrix[0][1] * matrix[0][1] / count;
  const float time_cross = rhs[1] - matrix[0][1] * sum / count;
  const float variance = energy - sum * sum / count -
      time_cross * time_cross / std::max(.001f, time_energy);
  for (int col = 0; col < 6; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 6; ++row)
      if (std::abs(matrix[row][col]) > std::abs(matrix[pivot][col])) pivot = row;
    if (std::abs(matrix[pivot][col]) < .001f) return false;
    for (int j = col; j < 7; ++j) std::swap(matrix[col][j], matrix[pivot][j]);
    const float divisor = matrix[col][col];
    for (int j = col; j < 7; ++j) matrix[col][j] /= divisor;
    for (int row = 0; row < 6; ++row) {
      if (row == col) continue;
      const float factor = matrix[row][col];
      for (int j = col; j < 7; ++j) matrix[row][j] -= factor * matrix[col][j];
    }
  }
  const float fundamental = matrix[2][6] * matrix[2][6] + matrix[3][6] * matrix[3][6];
  const float second = matrix[4][6] * matrix[4][6] + matrix[5][6] * matrix[5][6];
  const float fraction = fundamental / std::max(.001f, fundamental + second);
  ambiguous = fraction < .15f;
  float residual = energy;
  for (int i = 0; i < 6; ++i) residual -= matrix[i][6] * rhs[i];
  const float quality = std::max(0.f, std::min(1.f, 1 - residual / std::max(1.f, variance)));
  confidence = quality * fraction;
  if (ambiguous || fundamental < 2.25f || confidence < .35f || variance < count)
    return false;
  signal_phase = std::atan2(-matrix[3][6], matrix[2][6]);
  return std::isfinite(signal_phase);
}

void RssiPhaseTracker::update(uint32_t now_us, float rpm, bool phase_corrections) {
  if (!started) {
    last_update_us = now_us;
    started = true;
  }
  const uint32_t dt_us = now_us - last_update_us;
  if (have_rate) {
    const uint32_t age_before = last_update_us - last_rate_us;
    const uint32_t usable = age_before >= coast_us ? 0 : std::min(dt_us, coast_us - age_before);
    phase = wrap(phase + (nominal_rpm * rpm_to_rad_s + trim_rad_s) * usable * 1e-6f);
  }
  last_update_us = now_us;
  const bool fresh_rate = std::isfinite(rpm) && rpm >= 350 && rpm <= 4000;
  if (fresh_rate) {
    if (nominal_rpm > 0 && std::abs(rpm - nominal_rpm) > .2f * nominal_rpm) {
      trim_rad_s = 0;
      good_fits = 0;
    }
    nominal_rpm = rpm;
    last_rate_us = now_us;
    have_rate = true;
  }
  const bool rate_valid = have_rate && now_us - last_rate_us < coast_us;
  output.timestamp_us = now_us;
  output.phase_rad = phase;
  output.rpm = rate_valid ? nominal_rpm + trim_rad_s / rpm_to_rad_s : 0;
  output.confidence = output.error_rad = 0;
  output.status = rate_valid ? Status::Coasting : Status::Unavailable;
  if (!rate_valid) {
    good_fits = 0;
    trim_rad_s = 0;
    assisting = anchored = false;
    filtered_rpm = 0;
    return;
  }

  bool throttle_change = false;
  if (hint_seen) {
    if (!hint_initialized) {
      throttle_anchor = throttle_hint;
      hint_initialized = true;
    } else if (std::abs(throttle_hint - throttle_anchor) >= .01f) {
      throttle_anchor = throttle_hint;
      throttle_change = true;
    }
  }
  if (translating) { motion_seen = true; last_motion_us = now_us; }
  if (motion_seen && !translating && now_us - last_motion_us >= 500000)
    motion_seen = false;
  const bool motion_hold = motion_seen && now_us - last_motion_us < 500000;
  const float dt_s = std::min(.1f, dt_us * 1e-6f);
  if (!filtered_rpm) filtered_rpm = nominal_rpm;
  filtered_rpm += dt_s / (.08f + dt_s) * (nominal_rpm - filtered_rpm);

  if (motion_hold || !phase_corrections) {
    // A changed RF bearing must never be repaid as a delayed heading correction
    // after the sticks return to neutral. Relearn the reference without turning.
    assisting = anchored = candidate_seen = false;
    good_fits = 0;
    trim_rad_s = 0;
    output.rpm = nominal_rpm;
    output.status = motion_hold ? Status::MotionHold : Status::Coasting;
    return;
  }

  if (throttle_change) {
    if (!assisting && (!anchored || now_us - reference_us > 150000)) {
      anchored = false;
      good_fits = 0;
    }
    assisting = true;
    assist_start_us = settling_us = now_us;
    settling_rpm = filtered_rpm;
    rejected = false;
  }
  if (assisting) {
    if (std::abs(filtered_rpm - settling_rpm) >= std::max(15.f, .01f * settling_rpm)) {
      settling_rpm = filtered_rpm;
      settling_us = now_us;
    }
    if (now_us - assist_start_us >= 3000000 ||
        (now_us - assist_start_us >= 600000 && now_us - settling_us >= 350000)) {
      assisting = false;
      candidate_seen = false;
    }
  }
  if (!assisting) {
    // At steady speed RSSI is only an observation of the environment, not an
    // absolute heading sensor. Adapt its reference to DR, never the reverse.
    trim_rad_s = 0;
    output.rpm = nominal_rpm;
    output.status = Status::Monitoring;
  }
  const auto coast = [&] {
    trim_rad_s *= std::max(0.f, 1.f - float(dt_us) / 200000.f);
    output.rpm = nominal_rpm + trim_rad_s / rpm_to_rad_s;
  };
  if (now_us - last_fit_us < 10000) return;
  if (!size) { coast(); return; }
  const uint32_t newest = samples[(head + samples.size() - 1) % samples.size()].radio_us;
  if (observation_seen && newest == last_observation_radio_us) { coast(); return; }
  observation_seen = true;
  last_observation_radio_us = newest;
  const float fit_dt = std::min(.05f, float(now_us - last_fit_us) * 1e-6f);
  last_fit_us = now_us;
  float observed = 0, confidence = 0;
  uint32_t center_us = 0;
  bool ambiguous = false;
  if (!fit(nominal_rpm, now_us, observed, center_us, confidence, ambiguous)) {
    coast();
    good_fits = 0;
    output.confidence = confidence;
    if (ambiguous) output.status = Status::Ambiguous;
    return;
  }
  output.confidence = confidence;
  const float predicted = phase - output.rpm * rpm_to_rad_s * float(now_us - center_us) * 1e-6f;
  const float candidate = wrap(observed - predicted);
  const uint32_t span_us = uint32_t(std::max(40000.f, std::min(300000.f, 120000000.f / nominal_rpm)));
  if (!assisting || !anchored) {
    if (!good_fits || std::abs(wrap(candidate - candidate_reference)) > .25f) {
      acquisition_start_us = now_us;
      candidate_reference = candidate;
      good_fits = 1;
    } else {
      candidate_reference = wrap(candidate_reference + .25f * wrap(candidate - candidate_reference));
      ++good_fits;
    }
    // Require a full fit window of consistency, not three overlapping frames.
    if (good_fits >= 6 && now_us - acquisition_start_us >= span_us) {
      reference = candidate_reference;
      reference_us = now_us;
      anchored = true;
    }
    if (!assisting) {
      output.error_rad = anchored ? wrap(candidate - reference) : 0;
    } else {
      output.status = Status::Acquiring;
    }
    last_candidate = candidate;
    candidate_us = now_us;
    candidate_seen = true;
    return;
  }
  const float error = wrap(candidate - reference);
  output.error_rad = error;
  const bool discontinuity = candidate_seen && now_us - candidate_us < 120000 &&
                             std::abs(wrap(candidate - last_candidate)) > .3f;
  last_candidate = candidate;
  candidate_us = now_us;
  candidate_seen = true;
  // Consistent multipath can have excellent fit confidence. Persistence never
  // authorizes a large heading jump. Hold off for a complete independent window
  // after a discontinuity, and discard the integrator on suspect observations.
  if (std::abs(error) > .65f || discontinuity) {
    rejected = true;
    rejection_us = now_us;
    trim_rad_s = 0;
    output.rpm = nominal_rpm;
    output.status = Status::Rejected;
    return;
  }
  if (confidence < .45f || (rejected && now_us - rejection_us < span_us)) {
    coast();
    output.status = rejected ? Status::Rejected : Status::Coasting;
    return;
  }
  rejected = false;
  const float span_s = span_us * 1e-6f;
  const float kp = std::min(24.f, 4.f / span_s), ki = kp * kp / 4;
  phase = wrap(phase + std::max(-.12f, std::min(.12f, kp * fit_dt * error)));
  trim_rad_s += ki * fit_dt * error;
  const float max_trim = .03f * nominal_rpm * rpm_to_rad_s;
  trim_rad_s = std::max(-max_trim, std::min(max_trim, trim_rad_s));
  output.phase_rad = phase;
  output.rpm = nominal_rpm + trim_rad_s / rpm_to_rad_s;
  output.status = Status::Tracking;
}
