#include "rssi_rpm_estimator.h"
#include "rpm_heading_tracker.h"
#include "radio_clock.h"
#include <iostream>
#include <iomanip>
int main() {
  RssiRpmEstimator estimator;
  RpmHeadingTracker tracker;
  RadioClock clock;
  std::cout << std::setprecision(9);
  char kind;
  uint32_t t;
  while (std::cin >> kind >> t) {
    if (kind == 'r') {
      uint32_t local; int rssi;
      std::cin >> local >> rssi;
      clock.observe(t, local);
      estimator.add_sample(t, rssi);
    } else if (kind == 'u') {
      float throttle, target;
      std::cin >> throttle >> target;
      estimator.set_drive_hint(throttle, target, t / 1000);
      estimator.update(t / 1000);
      const auto &m = estimator.measurement();
      tracker.update(t, estimator.rpm(), {clock.local(m.timestamp_us), m.rpm, m.correlation});
      const auto &s = tracker.state();
      std::cout << s.heading_rad << ' ' << s.rpm << ' ' << s.error_rad << ' '
                << s.measurement_age_ms << ' ' << unsigned(s.status) << ' '
                << s.correction_rad << '\n';
    } else return 2;
  }
}
