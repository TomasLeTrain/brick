#include "rssi_rpm_estimator.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr uint32_t max_interpolation_gap_us = 8000;
constexpr uint32_t no_samples_timeout_ms = 50;
constexpr uint32_t analysis_interval_us = 50000;
constexpr float min_packet_rate_hz = 500;
constexpr float min_variance_db2 = 1;
constexpr float min_correlation = 0.55f;
constexpr float peak_tolerance = 0.08f;
constexpr float unavailable_correlation = -2;
constexpr float max_relative_change = 0.12f;
constexpr unsigned estimates_to_lock = 3;
} // namespace

void RssiRpmEstimator::reset() {
  head = size = 0;
  packet_count = observed_packet_count = 0;
  analyzed = false;
  agreeing_estimates = 0;
  previous_candidate_rpm = 0;
  estimated_rpm = packet_rate_hz = 0;
}

void RssiRpmEstimator::push(float rssi, bool valid) {
  history[head] = {rssi, packet_count, valid};
  head = (head + 1) % history_size;
  size = std::min(size + 1, history_size);
}

void RssiRpmEstimator::add_sample(uint32_t timestamp_us, int rssi_dbm) {
  if (rssi_dbm < -128 || rssi_dbm > 0)
    return;

  const uint32_t elapsed = timestamp_us - last_packet_us;
  if (size && (elapsed == 0 || elapsed >= 0x80000000UL))
    return; // Duplicate or out-of-order sample.
  if (size && elapsed >= no_samples_timeout_ms * 1000)
    reset();

  ++packet_count;
  if (!size) {
    last_grid_us = timestamp_us;
    push(rssi_dbm);
  } else {
    const bool interpolate = elapsed <= max_interpolation_gap_us;
    while (timestamp_us - last_grid_us >= sample_interval_us) {
      last_grid_us += sample_interval_us;
      if (interpolate) {
        const float fraction = float(last_grid_us - last_packet_us) / elapsed;
        push(last_rssi + fraction * (rssi_dbm - last_rssi));
      } else {
        // Leave holes where interpolation would span too much of a rotation.
        push(rssi_dbm, last_grid_us == timestamp_us);
      }
    }
  }
  last_packet_us = timestamp_us;
  last_rssi = rssi_dbm;
}

void RssiRpmEstimator::update(uint32_t now_ms) {
  if (!size)
    return;

  if (observed_packet_count != packet_count) {
    observed_packet_count = packet_count;
    last_sample_ms = now_ms;
  } else if (now_ms - last_sample_ms >= no_samples_timeout_ms) {
    reset();
    return;
  }

  if (size < history_size ||
      (analyzed && last_grid_us - last_analysis_us < analysis_interval_us))
    return;

  ++analyses;
  analyzed = true;
  last_analysis_us = last_grid_us;
  const int newest = (head + history_size - 1) % history_size;
  constexpr float window_seconds =
      (history_size - 1) * sample_interval_us / 1000000.0f;
  const uint32_t packets =
      history[newest].packet_count - history[head].packet_count;
  packet_rate_hz = packets / window_seconds;
  const float candidate_rpm =
      packet_rate_hz >= min_packet_rate_hz ? estimate() : 0;

  // Drop lock on abrupt changes, which can be jumps to a multiple of the
  // period. Require consecutive agreeing windows before using the new speed.
  if (candidate_rpm == 0) {
    agreeing_estimates = 0;
  } else if (std::abs(candidate_rpm - previous_candidate_rpm) >
             max_relative_change * previous_candidate_rpm) {
    agreeing_estimates = 1;
  } else if (agreeing_estimates < estimates_to_lock) {
    ++agreeing_estimates;
  }
  previous_candidate_rpm = candidate_rpm;
  estimated_rpm = agreeing_estimates >= estimates_to_lock ? candidate_rpm : 0;
}

float RssiRpmEstimator::estimate() const {
  // Fit and subtract a linear baseline using the observed portion of the span.
  std::array<float, history_size> window;
  std::array<bool, history_size> valid;
  constexpr float center = (history_size - 1) / 2.0f;
  int available = 0;
  float time_sum = 0, time_energy = 0, rssi_sum = 0, cross = 0;
  for (int i = 0; i < history_size; ++i) {
    const GridSample &sample = history[(head + i) % history_size];
    valid[i] = sample.valid;
    if (!sample.valid)
      continue;
    const float time = i - center;
    window[i] = sample.rssi + 64;
    ++available;
    time_sum += time;
    time_energy += time * time;
    rssi_sum += window[i];
    cross += time * window[i];
  }
  if (available < history_size * 4 / 5)
    return 0;

  const float slope = (cross - time_sum * rssi_sum / available) /
                      (time_energy - time_sum * time_sum / available);
  for (int i = 0; i < history_size; ++i) {
    if (valid[i])
      window[i] -= slope * (i - center);
  }

  // Compare a fixed recent segment with each earlier segment of equal length,
  // excluding holes from both sides. Normalize by their paired variances.
  std::array<float, max_lag + 2> correlation;
  constexpr int ref_start = max_lag + 1;
  for (int lag = first_lag - 1; lag <= max_lag + 1; ++lag) {
    int count = 0;
    float sum = 0, delayed_sum = 0, energy = 0, delayed_energy = 0, dot = 0;
    for (int i = ref_start; i < history_size; ++i) {
      if (!valid[i] || !valid[i - lag])
        continue;
      const float x = window[i];
      const float y = window[i - lag];
      ++count;
      sum += x;
      delayed_sum += y;
      energy += x * x;
      delayed_energy += y * y;
      dot += x * y;
    }

    correlation[lag] = unavailable_correlation;
    if (count < comparison_samples * 4 / 5)
      continue;
    energy -= sum * sum / count;
    delayed_energy -= delayed_sum * delayed_sum / count;
    const float min_energy = min_variance_db2 * count;
    if (energy >= min_energy && delayed_energy >= min_energy) {
      correlation[lag] = (dot - sum * delayed_sum / count) /
                         std::sqrt(energy * delayed_energy);
    }
  }

  const auto is_peak = [&correlation](int lag) {
    return correlation[lag - 1] != unavailable_correlation &&
           correlation[lag + 1] != unavailable_correlation &&
           correlation[lag] > correlation[lag - 1] &&
           correlation[lag] >= correlation[lag + 1];
  };
  float strongest = min_correlation;
  for (int lag = first_lag; lag <= max_lag; ++lag) {
    if (is_peak(lag))
      strongest = std::max(strongest, correlation[lag]);
  }

  // Integer multiples also correlate well. Use the first peak comparable to
  // the strongest, then fit the peak and its neighbors for sub-grid precision.
  const float threshold = std::max(min_correlation, strongest - peak_tolerance);
  for (int lag = first_lag; lag <= max_lag; ++lag) {
    if (!is_peak(lag) || correlation[lag] < threshold)
      continue;
    if (lag < min_lag)
      return 0;

    const float left = correlation[lag - 1];
    const float middle = correlation[lag];
    const float right = correlation[lag + 1];
    const float offset = 0.5f * (left - right) / (left - 2 * middle + right);
    const float refined_lag = lag + offset;
    return 60000000.0f / (refined_lag * sample_interval_us);
  }
  return 0;
}
