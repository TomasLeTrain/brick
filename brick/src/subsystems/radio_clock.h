#pragma once
#include <cstdint>

// Rolling minimum callback delay, excluding main-task queue latency. A bridge
// between Wi-Fi receive timestamps and micros(); all arithmetic wraps at 32 bits.
class RadioClock {
public:
  void reset() { seen = false; }
  void observe(uint32_t radio, uint32_t local) {
    const uint32_t candidate = local - radio;
    if (!seen) {
      offset = minimum = candidate;
      window = local;
      seen = true;
    }
    if (int32_t(candidate - offset) < 0) offset = candidate;
    if (int32_t(candidate - minimum) < 0) minimum = candidate;
    if (local - window >= 1000000) {
      offset = minimum;
      minimum = candidate;
      window = local;
    }
  }
  bool available() const { return seen; }
  uint32_t local(uint32_t radio) const { return radio + offset; }
private:
  bool seen = false;
  uint32_t offset = 0, minimum = 0, window = 0;
};
