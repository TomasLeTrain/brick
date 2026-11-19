#pragma once
#include <array>
#include <cstdint>

// RPM-only heading: revise the integral at timestamped measurement epochs, then
// project to now. This never uses RF waveform phase or throttle as a speed sensor.
class RpmHeadingTracker {
public:
  enum class Status : uint8_t { Unavailable = 8, DeadReckoning, Correcting, Coasting, Translation, TranslationTrend };
  struct TranslationTuning {
    float response_s = .2f;
    float max_correction_rad_s = 2.f;
    float prediction_s = .12f;
  };
  RpmHeadingTracker() = default;
  explicit RpmHeadingTracker(TranslationTuning tuning) : translation_tuning(tuning) {}
  void set_translation(bool commanded) { translation_requested = commanded; }
  struct Observation {
    uint32_t timestamp_us = 0; // Local clock: effective RPM measurement time.
    float rpm = 0, correlation = 0;
  };
  struct State {
    uint32_t timestamp_us = 0;
    float heading_rad = 0, rpm = 0, error_rad = 0, correction_rad = 0;
    float measurement_age_ms = 0, acceleration_rpm_s = 0, translation_blend = 0, translation_guard = 0;
    Status status = Status::Unavailable;
  };
  void update(uint32_t now_us, float measured_rpm, const Observation &observation,
              bool compensate = true);
  const State &state() const { return output; }
  void reset();
  static float wrap(float angle);
private:
  struct Speed { uint32_t time; float rpm; };
  std::array<Speed, 12> speeds{};
  unsigned head = 0, size = 0;
  bool started = false, have_rate = false, anchored = false;
  uint32_t last_update = 0, last_rate = 0, last_observation = 0, anchor_us = 0;
  float heading = 0, rate = 0, anchor_heading = 0, anchor_rpm = 0, prediction_rpm = 0, acceleration = 0, feedback_rad_s = 0, applied_feedback_rad_s = 0;
  TranslationTuning translation_tuning;
  std::array<Observation, 5> motion_samples{};
  unsigned motion_head = 0, motion_size = 0;
  bool source_seen = false, translation_requested = false, translation_seen = false, motion_noise_seen = false;
  uint32_t source_time = 0, last_translation = 0, last_motion_noise = 0;
  float translation_blend = 0;
  State output;
  Observation motion_observation(uint32_t now, const Observation &measurement);
  float guard_weight() const;
  void update_translation(uint32_t now, uint32_t dt_us);
  void observe(uint32_t now, const Observation &measurement);
  float slope(float &fitted_rpm) const;
  void projection(uint32_t age_us, float &angle, float &rpm) const;
};
