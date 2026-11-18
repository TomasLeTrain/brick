#include "rssi_rpm_estimator.h"
#include <cassert>
#include <cstdint>

struct RssiRpmEstimatorTestAccess {
  static void run() {
    RssiRpmEstimator estimator;
    estimator.estimated_rpm = 2000;
    estimator.last_good_ms = 1000;
    // Normal measurement noise passes; a large alias step in either direction
    // does not. Waiting longer after rejected candidates cannot unlock it.
    assert(estimator.plausible_change(2020, 1010));
    assert(!estimator.plausible_change(1000, 1010));
    assert(!estimator.plausible_change(3000, 1010));
    assert(!estimator.plausible_change(1000, 5000));
    assert(!estimator.plausible_change(3000, 5000));
    assert(estimator.plausible_change(2200, 1050));
    estimator.last_good_ms = UINT32_MAX - 9;
    assert(estimator.plausible_change(2050, 10));
    assert(!estimator.plausible_change(2500, 10));
  }
};

int main() { RssiRpmEstimatorTestAccess::run(); }
