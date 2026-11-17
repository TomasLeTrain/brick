#pragma once

#include <array>
#include <cstdint>

class RssiRpmEstimator {
public:
  // Wi-Fi receive timestamp in microseconds. The 32-bit clock may wrap.
  void add_sample(uint32_t timestamp_us, int rssi_dbm);
  // Call after draining the RSSI queue. now_ms is the local uptime clock.
  void update(uint32_t now_ms);
  void reset();

  uint32_t analysis_count() const { return analyses; }
  float rpm() const { return estimated_rpm; } // Zero when unavailable.
  float sample_rate_hz() const { return packet_rate_hz; }

private:
  static constexpr int sample_interval_us = 1000;
  static constexpr int comparison_samples = 250;
  static constexpr int min_lag = 18;
  static constexpr int max_lag = 155;
  // Check shorter periods too so their multiples aren't reported as in-range.
  static constexpr int first_lag = 9;
  static constexpr int history_size = comparison_samples + max_lag + 1;

  void push(float rssi, bool valid = true);
  float estimate() const;

  struct GridSample {
    float rssi;
    uint32_t packet_count;
    bool valid;
  };

  std::array<GridSample, history_size> history;
  int head = 0;
  int size = 0;
  uint32_t packet_count = 0;
  uint32_t observed_packet_count = 0;
  uint32_t last_sample_ms = 0;
  uint32_t last_packet_us = 0;
  uint32_t last_grid_us = 0;
  uint32_t last_analysis_us = 0;
  uint32_t analyses = 0;
  bool analyzed = false;
  unsigned agreeing_estimates = 0;
  float previous_candidate_rpm = 0;
  float last_rssi = 0;
  float estimated_rpm = 0;
  float packet_rate_hz = 0;
};
