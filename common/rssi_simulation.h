#pragma once

#include "run_dump.h"
#include <algorithm>
#include <cmath>

// Deterministic irregular packet timing with drops, noise and a second harmonic.
class RssiSimulation {
public:
  bool next(float rpm, uint32_t packet_rate = 1000, int noise_db = 3) {
    elapsed_us += std::max(1, int(1000000u / packet_rate) + noise(200));
    if (++packets % 9 == 0)
      return false;
    constexpr double pi = 3.14159265358979323846;
    const double phase = 2 * pi * rpm * elapsed_us / 60000000.0;
    rssi = std::lround(-64 + 8 * std::sin(phase) + 3 * std::cos(2 * phase) +
                      noise(noise_db));
    return true;
  }

  uint32_t time_us() const { return elapsed_us; }
  int rssi_dbm() const { return rssi; }

private:
  int noise(int range) {
    random_state = random_state * 1664525u + 1013904223u;
    return int((random_state >> 16) % (2 * range + 1)) - range;
  }
  uint32_t random_state = 12345, elapsed_us = 0, packets = 0;
  int rssi = -64;
};
