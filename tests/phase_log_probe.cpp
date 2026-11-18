#include "rssi_rpm_estimator.h"
#include "rssi_phase_tracker.h"
#include <iostream>
#include <iomanip>

// Old captures only contain task-delivery times. The caller supplies an
// approximate radio-to-local mapping; this cannot validate absolute timing.
int main() {
  RssiRpmEstimator rpm;
  RssiPhaseTracker phase, dead;
  std::cout << std::setprecision(9);
  char kind;
  uint32_t timestamp;
  while (std::cin >> kind >> timestamp) {
    if (kind == 'r') {
      uint32_t callback;
      int rssi;
      std::cin >> callback >> rssi;
      rpm.add_sample(timestamp, rssi);
      phase.add_sample(timestamp, callback, rssi);
    } else if (kind == 'u' || kind == 'p') {
      float throttle, target;
      float measured = 0;
      bool moving = false;
      if (kind == 'p') std::cin >> measured;
      std::cin >> throttle >> target;
      if (kind == 'p') std::cin >> moving;
      rpm.set_drive_hint(throttle, target, timestamp / 1000);
      rpm.update(timestamp / 1000);
      if (kind == 'u') measured = rpm.rpm();
      phase.set_drive_hint(throttle, moving);
      phase.update(timestamp, measured);
      dead.update(timestamp, measured, false);
      const auto &s = phase.state();
      std::cout << s.phase_rad << ' ' << s.rpm << ' ' << s.confidence
                << ' ' << unsigned(s.status) << ' ' << s.error_rad << ' ' << dead.state().phase_rad << '\n';
    } else return 2;
  }
}
