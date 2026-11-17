#include "rssi_rpm_estimator.h"
#include <array>
#include <iomanip>
#include <iostream>

int main() {
  RssiRpmEstimator estimator;
  uint32_t t, now = 0, last_update = 0;
  int rssi;
  RssiRpmEstimator::Diagnostics diagnostic;
  std::array<float, RssiRpmEstimator::max_lag + 2> fast{}, slow{};
  while (std::cin >> t >> rssi) {
    estimator.add_sample(t, rssi);
    now += 1;
    if (now - last_update < 10)
      continue;
    last_update = now;
    const auto count = estimator.analysis_count();
    estimator.update(now);
    if (estimator.analysis_count() != count) {
      diagnostic = estimator.diagnostics();
      estimator.correlation_curve(diagnostic.comparison_samples,
                                   diagnostic.min_lag, diagnostic.max_lag, fast.data());
      estimator.correlation_curve(250, 9, 155, slow.data());
    }
  }
  std::cout << diagnostic.timestamp_us << ' ' << diagnostic.comparison_samples
            << ' ' << diagnostic.min_lag << ' ' << diagnostic.max_lag << '\n';
  std::cout << std::setprecision(9);
  for (int i = 0; i < int(fast.size()); ++i)
    std::cout << i << ' ' << fast[i] << ' ' << slow[i] << '\n';
}
