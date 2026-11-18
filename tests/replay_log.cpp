#include "rssi_rpm_estimator.h"
#include <iostream>

struct RssiRpmEstimatorTestAccess {
  static void restore_lock(RssiRpmEstimator &estimator, float rpm, uint32_t now_ms) {
    estimator.estimated_rpm = rpm;
    estimator.locked = true;
    estimator.last_good_ms = now_ms;
    estimator.diagnostic.fast_rpm = rpm;
    estimator.diagnostic.hint_used = false;
    estimator.transient_started = true;
    estimator.transient_started_ms = now_ms;
  }
};

int main() {
  RssiRpmEstimator estimator;
  char kind;
  uint32_t timestamp;
  while (std::cin >> kind >> timestamp) {
    if (kind == 'r') {
      int rssi;
      std::cin >> rssi;
      estimator.add_sample(timestamp, rssi);
    } else if (kind == 'l') {
      float rpm;
      std::cin >> rpm;
      RssiRpmEstimatorTestAccess::restore_lock(estimator, rpm, timestamp / 1000);
    } else if (kind == 'u') {
      float throttle, target;
      std::cin >> throttle >> target;
      estimator.set_drive_hint(throttle, target, timestamp / 1000);
      estimator.update(timestamp / 1000);
      std::cout << estimator.rpm() << ' '
                << unsigned(estimator.diagnostics().status) << '\n';
    } else {
      return 2;
    }
  }
}
