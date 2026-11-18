#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

// A bounded, nonblocking mailbox: payload fields are atomic too, so a failed
// read is safe under the C++ memory model. The fast task never spins if the
// general task is preempted halfway through a publication.
class HeadingClock {
public:
  void publish(uint32_t timestamp_us, float phase_rad, float rpm) {
    sequence.fetch_add(1);
    timestamp.store(timestamp_us);
    phase.store(phase_rad);
    rate.store(rpm);
    sequence.fetch_add(1);
  }
  float render(uint32_t now_us, float turn_rad_s) {
    const uint32_t version = sequence.load();
    if (version && !(version & 1u)) {
      const uint32_t candidate_time = timestamp.load();
      const float candidate_phase = phase.load(), candidate_rate = rate.load();
      if (sequence.load() == version && std::isfinite(candidate_phase) &&
          std::isfinite(candidate_rate) && candidate_rate >= 0 && candidate_rate <= 5000) {
        cached_time = candidate_time;
        cached_phase = candidate_phase;
        cached_rate = candidate_rate;
      }
    }
    const uint32_t elapsed = now_us - last_render_us;
    if (rendered && elapsed <= 50000 && std::isfinite(turn_rad_s))
      heading_offset = positive(heading_offset + turn_rad_s * elapsed * 1e-6f);
    rendered = true;
    last_render_us = now_us;
    const uint32_t age = int32_t(now_us - cached_time) < 0 ? 0 :
                         std::min(now_us - cached_time, uint32_t(300000));
    const float angle = positive(cached_phase + cached_rate * (tau / 60) * age * 1e-6f - heading_offset);
    last_angle.store(angle);
    return angle;
  }
  float angle() const { return last_angle.load(); }

private:
  static constexpr float tau = 6.2831853071795864769f;
  static float positive(float value) {
    value = std::fmod(value, tau);
    return value < 0 ? value + tau : value;
  }
  std::atomic<uint32_t> sequence{0}, timestamp{0};
  std::atomic<float> phase{0}, rate{0}, last_angle{0};
  uint32_t cached_time = 0, last_render_us = 0;
  float cached_phase = 0, cached_rate = 0, heading_offset = 0;
  bool rendered = false;
};
