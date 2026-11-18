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
  analyzed = long_analyzed = locked = false;
  command_seen = transient_started = false;
  agreeing_estimates = stable_long_estimates = 0;
  previous_candidate_rpm = previous_long_rpm = 0;
  estimated_rpm = packet_rate_hz = expected_rpm = 0;
  long_result = {};
  diagnostic = {};
}

void RssiRpmEstimator::set_drive_hint(float throttle, float steady_rpm,
                                     uint32_t now_ms) {
  if (!std::isfinite(throttle) || !std::isfinite(steady_rpm))
    return;
  if (!command_seen || std::abs(throttle - command_throttle) >= 0.01f) {
    command_throttle = throttle;
    command_changed_ms = transient_started_ms = now_ms;
    transient_started = command_seen;
    command_seen = true;
  }
  expected_rpm = steady_rpm > 0 ? steady_rpm : 0;
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
    diagnostic.status = Status::Stale;
    return;
  }
  if (size < history_size ||
      (analyzed && last_grid_us - last_analysis_us < 10000))
    return;

  analyzed = true;
  ++analyses;
  last_analysis_us = last_grid_us;
  diagnostic.timestamp_us = last_grid_us;
  const int newest = (head + history_size - 1) % history_size;
  constexpr float window_seconds = (history_size - 1) / 1000.0f;
  packet_rate_hz = (history[newest].packet_count - history[head].packet_count) /
                   window_seconds;
  if (packet_rate_hz < min_packet_rate_hz) {
    estimated_rpm = 0;
    locked = false;
    agreeing_estimates = 0;
    diagnostic.status = Status::LowPacketRate;
    return;
  }

  const bool quiet_command = command_seen && now_ms - command_changed_ms >= 600;
  if (!long_analyzed || last_grid_us - last_long_us >= analysis_interval_us) {
    const bool consistent = !locked || diagnostic.fast_rpm <= 0 ||
        std::abs(diagnostic.fast_rpm - long_result.rpm) < 0.08f * long_result.rpm;
    diagnostic.hint_used = quiet_command && expected_rpm > 0 &&
        (stable_long_estimates >= 3 || diagnostic.hint_used) && consistent;
    long_result = analyze(comparison_samples, first_lag, max_lag,
                          diagnostic.hint_used ? expected_rpm : 0);
    diagnostic.long_rpm = long_result.rpm;
    stable_long_estimates =
        long_result.rpm > 0 && previous_long_rpm > 0 &&
        std::abs(long_result.rpm - previous_long_rpm) < 0.03f * previous_long_rpm
            ? std::min(stable_long_estimates + 1, 4u) : 0;
    previous_long_rpm = long_result.rpm;
    last_long_us = last_grid_us;
    long_analyzed = true;

    // A settled feedforward hint may resolve a different fundamental from
    // the same strong long-window peaks, outside the local tracking search.
    if (locked && diagnostic.hint_used && long_result.rpm > 0 &&
        long_result.correlation >= 0.70f &&
        std::abs(long_result.rpm - estimated_rpm) > 0.25f * estimated_rpm) {
      estimated_rpm = long_result.rpm;
      last_good_ms = now_ms;
    }
    if (!locked) {
      agreeing_estimates =
          long_result.rpm > 0 && previous_candidate_rpm > 0 &&
          std::abs(long_result.rpm - previous_candidate_rpm) <
              max_relative_change * previous_candidate_rpm
              ? agreeing_estimates + 1 : (long_result.rpm > 0 ? 1 : 0);
      previous_candidate_rpm = long_result.rpm;
      diagnostic.mode = 0;
      diagnostic.status = long_result.rpm > 0 ? Status::Confirming : long_result.status;
      diagnostic.fast_rpm = 0;
      diagnostic.comparison_samples = comparison_samples;
      diagnostic.min_lag = first_lag;
      diagnostic.max_lag = max_lag;
      diagnostic.correlation = long_result.correlation;
      if (agreeing_estimates >= estimates_to_lock) {
        estimated_rpm = long_result.rpm;
        last_good_ms = now_ms;
        locked = true;
      }
    }
  }
  if (!locked)
    return;

  bool transient = transient_started && now_ms - transient_started_ms < 300;
  const float period = 60000.0f / estimated_rpm; // Grid is 1 ms.
  const int samples = std::max(transient ? 40 : 60,
      std::min(comparison_samples, int(std::lround(period * (transient ? 2 : 3)))));
  const float band = transient ? 0.45f : 0.25f;
  const int first = std::max(first_lag, int(std::floor(period * (1 - band))));
  const int last = std::min(max_lag, int(std::ceil(period * (1 + band))));
  const Result fast = analyze(samples, first, last, estimated_rpm);
  diagnostic.fast_rpm = fast.rpm;
  diagnostic.comparison_samples = samples;
  diagnostic.min_lag = first;
  diagnostic.max_lag = last;
  diagnostic.mode = transient ? 2 : 1;
  diagnostic.correlation = fast.correlation;
  diagnostic.status = fast.status;

  if (fast.rpm > 0 && fast.correlation >= 0.60f) {
    estimated_rpm = fast.rpm;
    diagnostic.status = Status::Tracking;
    last_good_ms = now_ms;
    if (long_result.rpm > 0 &&
        std::abs(fast.rpm - long_result.rpm) > 0.06f * long_result.rpm) {
      transient_started = true;
      transient_started_ms = now_ms;
    }
  } else if (!transient && long_result.rpm > 0 &&
             long_result.correlation >= 0.70f &&
             std::abs(long_result.rpm - estimated_rpm) < 0.08f * estimated_rpm) {
    estimated_rpm = long_result.rpm;
    diagnostic.status = Status::Tracking;
    last_good_ms = now_ms;
    diagnostic.comparison_samples = comparison_samples;
    diagnostic.min_lag = first_lag;
    diagnostic.max_lag = max_lag;
    diagnostic.correlation = long_result.correlation;
  } else if (now_ms - last_good_ms >= 150) {
    estimated_rpm = 0;
    locked = false;
    agreeing_estimates = 0;
    diagnostic.status = Status::Stale;
  } else {
    diagnostic.status = Status::Holding;
  }
}

void RssiRpmEstimator::correlation_curve(int samples, int first, int last,
                                        float *curve) const {
  std::fill(curve, curve + max_lag + 2, unavailable_correlation);
  analyze(samples, first, last, 0, curve);
}

RssiRpmEstimator::Result RssiRpmEstimator::analyze(
    int samples, int first, int last, float preferred_rpm, float *output) const {
  if (samples < 2 || samples > comparison_samples || first < first_lag ||
      last > max_lag || first > last || size < samples + last + 1)
    return {0, 0, Status::MissingData};
  const int span = samples + last + 1;
  const int oldest = (head + history_size - span) % history_size;
  // Fit and subtract a linear baseline using the observed portion of the span.
  std::array<float, history_size> window;
  std::array<bool, history_size> valid;
  const float center = (span - 1) / 2.0f;
  int available = 0;
  float time_sum = 0, time_energy = 0, rssi_sum = 0, cross = 0;
  for (int i = 0; i < span; ++i) {
    const GridSample &sample = history[(oldest + i) % history_size];
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
  if (available < span * 4 / 5)
    return {0, 0, Status::MissingData};

  const float slope = (cross - time_sum * rssi_sum / available) /
                      (time_energy - time_sum * time_sum / available);
  for (int i = 0; i < span; ++i) {
    if (valid[i])
      window[i] -= slope * (i - center);
  }

  // Compare a fixed recent segment with each earlier segment of equal length,
  // excluding holes from both sides. Normalize by their paired variances.
  std::array<float, max_lag + 2> correlation;
  correlation.fill(unavailable_correlation);
  const int ref_start = last + 1;
  bool enough_pairs = false, enough_variance = false;
  for (int lag = first - 1; lag <= last + 1; ++lag) {
    int count = 0;
    float sum = 0, delayed_sum = 0, energy = 0, delayed_energy = 0, dot = 0;
    for (int i = ref_start; i < span; ++i) {
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
    if (count < samples * 4 / 5)
      continue;
    enough_pairs = true;
    energy -= sum * sum / count;
    delayed_energy -= delayed_sum * delayed_sum / count;
    const float min_energy = min_variance_db2 * count;
    if (energy >= min_energy && delayed_energy >= min_energy) {
      enough_variance = true;
      correlation[lag] = (dot - sum * delayed_sum / count) /
                         std::sqrt(energy * delayed_energy);
    }
  }

  if (output)
    std::copy(correlation.begin(), correlation.end(), output);
  if (!enough_pairs)
    return {0, 0, Status::MissingData};
  if (!enough_variance)
    return {0, 0, Status::LowVariance};

  const auto is_peak = [&correlation](int lag) {
    return correlation[lag - 1] != unavailable_correlation &&
           correlation[lag + 1] != unavailable_correlation &&
           correlation[lag] > correlation[lag - 1] &&
           correlation[lag] >= correlation[lag + 1];
  };
  float strongest = min_correlation;
  for (int lag = first; lag <= last; ++lag) {
    if (is_peak(lag))
      strongest = std::max(strongest, correlation[lag]);
  }

  // Choose among strong peaks. A prior can break a near-equal correlation
  // tie, but cannot create a candidate that the signal does not support.
  const float threshold = std::max(min_correlation, strongest - peak_tolerance);
  int selected = 0;
  float distance = 1e30f;
  for (int lag = first; lag <= last; ++lag) {
    if (!is_peak(lag) || correlation[lag] < threshold)
      continue;
    if (lag < min_lag)
      return {};
    if (!selected)
      selected = lag;
    if (preferred_rpm > 0) {
      const float candidate = 60000.0f / lag;
      const float error = std::abs(std::log(candidate / preferred_rpm));
      // During global acquisition a feedforward hint can resolve a two-lobe
      // ambiguity. It cannot pull a good measurement to arbitrary multiples.
      const float ratio = float(selected) / lag;
      const bool global_search = first == first_lag && last == max_lag;
      const bool compatible = !global_search ||
          (error < std::log(1.20f) && (ratio > 0.90f ||
                                     (ratio > 0.45f && ratio < 0.55f)));
      if (compatible && error < distance) {
        distance = error;
        selected = lag;
      }
    } else {
      break;
    }
  }
  if (!selected)
    return {};
  const float left = correlation[selected - 1];
  const float middle = correlation[selected];
  const float right = correlation[selected + 1];
  const float offset = 0.5f * (left - right) / (left - 2 * middle + right);
  return {60000.0f / (selected + offset), middle, Status::Tracking};
}
