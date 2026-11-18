#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Continuous RPM dead reckoning, with RSSI phase assistance during commanded
// speed transitions. RF monitoring never changes steady-speed heading.
class RssiPhaseTracker {
public:
  enum class Status : uint8_t {
    Unavailable, Acquiring, Tracking, Coasting, Ambiguous,
    Monitoring, MotionHold, Rejected
  };
  struct State {
    uint32_t timestamp_us = 0;
    float phase_rad = 0, rpm = 0, error_rad = 0, confidence = 0;
    Status status = Status::Unavailable;
  };
  void add_sample(uint32_t radio_us, uint32_t callback_us, int rssi_dbm);
  void update(uint32_t now_us, float measured_rpm, bool phase_corrections = true);
  // Normalized motor throttle and actual translation intent, after deadzone.
  // A hint opens assistance on meaningful throttle changes, never on stick noise.
  void set_drive_hint(float throttle, bool translating = false);
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
  float candidate_reference = 0;
  uint32_t acquisition_start_us = 0;
  unsigned good_fits = 0;
  bool hint_seen = false, hint_initialized = false, translating = false;
  bool motion_seen = false, assisting = false;
  float throttle_hint = 0, throttle_anchor = 0;
  float filtered_rpm = 0, settling_rpm = 0;
  uint32_t last_motion_us = 0, assist_start_us = 0, settling_us = 0;
  uint32_t reference_us = 0;
  bool candidate_seen = false, rejected = false;
  uint32_t candidate_us = 0, rejection_us = 0;
  float last_candidate = 0;
  State output;
};
