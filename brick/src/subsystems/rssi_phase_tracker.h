#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Local-clock rotor phase, with a repeatable RSSI fundamental as its reference.
// First lock preserves the integrated heading; no compass direction is implied.
class RssiPhaseTracker {
public:
  enum class Status : uint8_t { Unavailable, Acquiring, Tracking, Coasting, Ambiguous };
  struct State {
    uint32_t timestamp_us = 0;
    float phase_rad = 0, rpm = 0, error_rad = 0, confidence = 0;
    Status status = Status::Unavailable;
  };
  void add_sample(uint32_t radio_us, uint32_t callback_us, int rssi_dbm);
  void update(uint32_t now_us, float measured_rpm, bool phase_corrections = true);
  void reset();
  const State &state() const { return output; }
  static float wrap(float angle);

private:
  struct Sample { uint32_t radio_us; float rssi; };
  bool fit(float rpm, uint32_t now_us, float &signal_phase, uint32_t &center_us,
           float &confidence, bool &ambiguous) const;
  std::array<Sample, 512> samples{};
  size_t head = 0, size = 0;
  bool clock_seen = false, started = false, have_rate = false, anchored = false;
  uint32_t last_radio_us = 0, last_callback_us = 0;
  uint32_t clock_offset_us = 0, window_offset_us = 0, clock_window_us = 0;
  uint32_t last_rate_us = 0, last_update_us = 0, last_fit_us = 0;
  uint32_t last_observation_radio_us = 0;
  bool observation_seen = false;
  float phase = 0, nominal_rpm = 0, trim_rad_s = 0, reference = 0;
  float candidate_reference = 0, previous_error = 0;
  float acquisition_delta = 0;
  uint32_t acquisition_start_us = 0;
  unsigned good_fits = 0, large_error_fits = 0;
  State output;
};
