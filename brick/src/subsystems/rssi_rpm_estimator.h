#pragma once

#include <array>
#include <cstdint>

class RssiRpmEstimator {
public:
  static constexpr int max_lag = 155;
  enum class Status : uint8_t {
    Warmup, LowPacketRate, LowVariance, MissingData, NoPeak,
    Confirming, Tracking, Holding, Stale, JumpRejected
  };
  struct Diagnostics {
    float fast_rpm = 0, long_rpm = 0, correlation = 0;
    uint32_t timestamp_us = 0;
    uint16_t comparison_samples = 250, min_lag = 9, max_lag = 155;
    uint8_t mode = 0; // 0 acquisition, 1 tracking, 2 transient.
    bool hint_used = false;
    Status status = Status::Warmup;
  };

  struct Measurement {
    uint32_t timestamp_us = 0; // Effective midpoint, in the Wi-Fi packet clock.
    float rpm = 0, correlation = 0;
  };
  const Measurement &measurement() const { return accepted_measurement; }

  void add_sample(uint32_t timestamp_us, int rssi_dbm);
  // Motor throttle and its calibrated steady-state RPM. The hint only chooses
  // between credible correlation peaks after command/measurement stability.
  void set_drive_hint(float throttle, float steady_rpm, uint32_t now_ms);
  void update(uint32_t now_ms);
  void reset();

  uint32_t analysis_count() const { return analyses; }
  float rpm() const { return estimated_rpm; }
  float sample_rate_hz() const { return packet_rate_hz; }
  const Diagnostics &diagnostics() const { return diagnostic; }

  // Desktop/test inspection of the same correlation calculation, no state change.
  void correlation_curve(int samples, int first, int last, float *curve) const;

private:
  friend struct RssiRpmEstimatorTestAccess; // Replay a recorded lock with live RSSI history.
  static constexpr int sample_interval_us = 1000;
  static constexpr int comparison_samples = 250;
  static constexpr int min_lag = 18, first_lag = 9;
  static constexpr int history_size = comparison_samples + max_lag + 1;

  struct GridSample {
    float rssi;
    uint32_t packet_count;
    bool valid;
  };
  struct Result {
    float rpm = 0, correlation = 0;
    Status status = Status::NoPeak;
    uint32_t timestamp_us = 0;
  };
  void push(float rssi, bool valid = true);
  void accept(const Result &result);
  bool plausible_change(float candidate_rpm, uint32_t now_ms) const;
  Result analyze(int samples, int first, int last, float preferred_rpm,
                 float *output = nullptr) const;

  std::array<GridSample, history_size> history;
  int head = 0, size = 0;
  uint32_t packet_count = 0, observed_packet_count = 0, last_sample_ms = 0;
  uint32_t last_packet_us = 0, last_grid_us = 0, last_analysis_us = 0;
  uint32_t last_long_us = 0, last_good_ms = 0, analyses = 0;
  uint32_t command_changed_ms = 0, transient_started_ms = 0;
  bool analyzed = false, long_analyzed = false, command_seen = false;
  bool transient_started = false, locked = false;
  unsigned agreeing_estimates = 0, stable_long_estimates = 0;
  unsigned conflicting_long_estimates = 0;
  float conflicting_long_rpm = 0;
  float previous_candidate_rpm = 0, previous_long_rpm = 0;
  float command_throttle = 0, expected_rpm = 0, last_rssi = 0;
  float estimated_rpm = 0, packet_rate_hz = 0;
  Measurement accepted_measurement;
  Result long_result;
  Diagnostics diagnostic;
};
