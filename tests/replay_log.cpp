#include "rssi_rpm_estimator.h"
#include <iostream>

int main() {
  RssiRpmEstimator estimator;
  char kind;
  uint32_t timestamp;
  while (std::cin >> kind >> timestamp) {
    if (kind == 'r') {
      int rssi;
      std::cin >> rssi;
      estimator.add_sample(timestamp, rssi);
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
