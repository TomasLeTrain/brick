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
  observation_seen = false;
  phase = nominal_rpm = trim_rad_s = reference = 0;
  good_fits = large_error_fits = 0;
  output = {};
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
  if (!rate_valid) { good_fits = 0; trim_rad_s = 0; return; }
  const auto coast = [&] {
    trim_rad_s *= std::max(0.f, 1.f - float(dt_us) / 200000.f);
    output.rpm = nominal_rpm + trim_rad_s / rpm_to_rad_s;
  };
  if (!phase_corrections) {
    trim_rad_s = 0;
    output.rpm = nominal_rpm;
    return;
  }
  // Never reuse the same observation, or call the expensive fit per packet.
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
    good_fits = large_error_fits = 0;
    output.confidence = confidence;
    output.status = ambiguous ? Status::Ambiguous : Status::Coasting;
    return;
  }
  output.confidence = confidence;
  const float predicted = phase - output.rpm * rpm_to_rad_s * float(now_us - center_us) * 1e-6f;
  if (!anchored) {
    const float candidate = wrap(observed - predicted);
    if (!good_fits || std::abs(wrap(candidate - candidate_reference)) > .25f) {
      candidate_reference = candidate;
      acquisition_start_us = now_us;
      acquisition_delta = 0;
      good_fits = 1;
    } else {
      acquisition_delta += wrap(candidate - candidate_reference);
      candidate_reference = candidate;
      ++good_fits;
    }
    output.status = Status::Acquiring;
    if (good_fits >= 6) {
      // Estimate the initial frequency offset from several phase observations.
      // Project the first reference to now so learning a small RPM bias does
      // not itself move the heading by half a fit window on initial lock.
      const float limit = confidence >= .65f ? .02f * nominal_rpm * rpm_to_rad_s : 0;
      trim_rad_s = std::max(-limit, std::min(limit,
          acquisition_delta / (float(now_us - acquisition_start_us) * 1e-6f)));
      reference = wrap(candidate_reference + trim_rad_s * float(now_us - center_us) * 1e-6f);
      anchored = true;
      output.rpm = nominal_rpm + trim_rad_s / rpm_to_rad_s;
      output.status = Status::Tracking;
    }
    return;
  }
  const float error = wrap(observed - reference - predicted);
  output.error_rad = error;
  if (std::abs(error) > 1.f) {
    large_error_fits = std::abs(wrap(error - previous_error)) < .25f ? large_error_fits + 1 : 1;
    previous_error = error;
    if (large_error_fits < 3) { output.status = Status::Acquiring; return; }
  } else {
    large_error_fits = 0;
  }
  const float span_s = std::max(.04f, std::min(.3f, 120.f / nominal_rpm));
  const float kp = std::min(24.f, 4.f / span_s), ki = kp * kp / 4;
  phase = wrap(phase + std::max(-.12f, std::min(.12f, kp * fit_dt * error)));
  trim_rad_s += ki * fit_dt * error;
  const float max_trim = .03f * nominal_rpm * rpm_to_rad_s;
  trim_rad_s = std::max(-max_trim, std::min(max_trim, trim_rad_s));
  output.phase_rad = phase;
  output.rpm = nominal_rpm + trim_rad_s / rpm_to_rad_s;
  output.status = Status::Tracking;
}
