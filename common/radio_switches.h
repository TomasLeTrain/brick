#pragma once
#include <cstdint>

// SWC is a three-position switch: the middle retains the last endpoint's mode.
// Ignore disconnected/failsafe frames and values outside the channel range.
class MaxThrottleSwitch {
public:
  void observe(uint16_t channel, bool connected) {
    if (!connected || channel < 1000 || channel > 2000)
      return;
    if (channel <= 1100)
      enabled = false;
    else if (channel >= 1900)
      enabled = true;
  }
  bool active() const { return enabled; }

private:
  bool enabled = false;
};
