#include "rssi_rpm_estimator.h"
#include "rssi_phase_tracker.h"
#include <iostream>

// Old captures only contain task-delivery times. The caller supplies an
// approximate radio-to-local mapping; this cannot validate absolute timing.
int main() {
  RssiRpmEstimator rpm;
  RssiPhaseTracker phase;
  char kind;
  uint32_t timestamp;
  while (std::cin >> kind >> timestamp) {
    if (kind == 'r') {
      uint32_t callback;
      int rssi;
      std::cin >> callback >> rssi;
      rpm.add_sample(timestamp, rssi);
      phase.add_sample(timestamp, callback, rssi);
    } else if (kind == 'u') {
      float throttle, target;
      std::cin >> throttle >> target;
      rpm.set_drive_hint(throttle, target, timestamp / 1000);
      rpm.update(timestamp / 1000);
      phase.update(timestamp, rpm.rpm());
      const auto &s = phase.state();
      std::cout << s.phase_rad << ' ' << s.rpm << ' ' << s.confidence
                << ' ' << unsigned(s.status) << '\n';
    } else return 2;
  }
}
