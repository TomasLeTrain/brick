#include "rssi_rpm_estimator.h"
#include <cassert>
#include <cmath>
int main() {
  for (float rpm : {400.f, 1500.f, 3000.f}) {
    RssiRpmEstimator estimator;
    const uint32_t origin = UINT32_MAX - 300000;
    unsigned accepted = 0;
    for (uint32_t t = 0; t < 1500000; t += 1000) {
      estimator.add_sample(origin + t, std::lround(-64 + 10 * std::sin(t * rpm * 6.283185307179586 / 60000000)));
      if (t % 10000) continue;
      estimator.set_drive_hint(.3f, rpm, t / 1000);
      estimator.update(t / 1000);
      const auto m = estimator.measurement();
      if (m.rpm <= 0) continue;
      const auto &d = estimator.diagnostics();
      assert(std::abs(m.rpm - rpm) < .025f * rpm);
      const float delay = ((d.comparison_samples - 1) * 1000 + 60000000.f / m.rpm) * .5f;
      assert(std::abs(float(uint32_t(d.timestamp_us - m.timestamp_us)) - delay) < 1000);
      estimator.update(t / 1000 + 1); // No new radio sample: no new epoch.
      assert(estimator.measurement().timestamp_us == m.timestamp_us);
      ++accepted;
    }
    assert(accepted > 50);
  }
}
