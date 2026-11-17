#include "rssi_rpm_estimator.h"

#include <cassert>
#include <climits>
#include <cmath>
#include <cstdint>

namespace {
constexpr double pi = 3.14159265358979323846;

struct Signal {
  uint32_t random_state = 12345;

  int noise(int range) {
    random_state = random_state * 1664525u + 1013904223u;
    return int((random_state >> 16) % (2 * range + 1)) - range;
  }

  void feed(RssiRpmEstimator &estimator, float rpm, uint32_t origin,
            uint32_t start_us, uint32_t duration_us, uint32_t spacing_us = 1000,
            int fundamental = 8, int harmonic = 3) {
    unsigned packet = 0;
    for (uint32_t t = start_us; t < start_us + duration_us;
         t += spacing_us + noise(250)) {
      if (++packet % 9 == 0)
        continue; // Simulate dropped packets as well as timing jitter.

      const double phase = 2 * pi * rpm * t / 60000000.0;
      const double rssi = -64 + 4 * (t / 1000000.0) +
                          fundamental * std::sin(phase) +
                          harmonic * std::cos(2 * phase) + noise(3);
      estimator.add_sample(origin + t, std::lround(rssi));
      estimator.add_sample(origin + t, -10);     // Duplicate timestamp.
      estimator.add_sample(origin + t - 1, -10); // Out-of-order timestamp.
      estimator.add_sample(origin + t + 1, INT_MIN);
      estimator.update(t / 1000);
    }
  }
};

void expect_rpm(const RssiRpmEstimator &estimator, float rpm) {
  assert(std::abs(estimator.rpm() - rpm) <= rpm * 0.025f);
  assert(estimator.sample_rate_hz() >= 750);
}
} // namespace

int main() {
  for (float rpm : {400.f, 600.f, 1111.f, 2275.f, 3000.f}) {
    RssiRpmEstimator estimator;
    Signal signal;
    const uint32_t origin = UINT32_MAX - 200000;
    signal.feed(estimator, rpm, origin, 0, 450000);
    assert(estimator.rpm() == 0);
    signal.feed(estimator, rpm, origin, 450000, 850000);
    expect_rpm(estimator, rpm);

    estimator.update(1350);
    assert(estimator.rpm() == 0);
    signal.feed(estimator, rpm, origin, 1400000, 600000);
    expect_rpm(estimator, rpm);
    // A short burst loss leaves a hole, and the next window can still lock.
    signal.feed(estimator, rpm, origin, 2020000, 100000);
    expect_rpm(estimator, rpm);
    estimator.add_sample(origin + 2200000, -60);
    estimator.update(2200);
    assert(estimator.rpm() == 0);
  }

  for (float rpm : {0.f, 250.f, 3800.f}) {
    RssiRpmEstimator estimator;
    Signal{}.feed(estimator, rpm, 0, 0, 1000000, 1000, 8, 0);
    assert(estimator.rpm() == 0);
  }

  RssiRpmEstimator harmonic;
  Signal{}.feed(harmonic, 1390, 0, 0, 1000000, 1000, 4, 8);
  expect_rpm(harmonic, 1390);

  RssiRpmEstimator sparse;
  Signal{}.feed(sparse, 3000, 0, 0, 1000000, 3000);
  assert(sparse.sample_rate_hz() > 0 && sparse.sample_rate_hz() < 500);
  assert(sparse.rpm() == 0);
}
