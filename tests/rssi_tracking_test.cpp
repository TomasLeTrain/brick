#include "rssi_rpm_estimator.h"
#include "../common/rssi_simulation.h"
#include <cassert>
#include <cmath>

int main() {
  // Spin-up and braking, with and without a throttle cue. The signal phase is
  // continuous and packet timing includes jitter and dropped samples.
  for (bool cue : {false, true}) {
    for (bool braking : {false, true}) {
      RssiRpmEstimator estimator;
      RssiSimulation signal;
      const float start = braking ? 3000 : 1200;
      const float end = braking ? 1200 : 3000;
      uint32_t last_update = 0;
      float absolute_error = 0;
      unsigned checks = 0;
      while (signal.time_us() < 3000000) {
        const float rpm = RssiSimulation::ramp_rpm(
            signal.time_us(), start, end, 3000000);
        if (!signal.next(rpm))
          continue;
        estimator.add_sample(signal.time_us(), signal.rssi_dbm());
        if (signal.time_us() - last_update < 10000)
          continue;
        last_update = signal.time_us();
        const float target = cue && last_update >= 900000 ? end : start;
        estimator.set_drive_hint(target / 6784, target, last_update / 1000);
        estimator.update(last_update / 1000);
        if (last_update >= 1000000 && last_update < 2400000) {
          absolute_error += std::abs(estimator.rpm() - rpm);
          ++checks;
        }
      }
      assert(checks > 100);
      assert(absolute_error / checks < 100);
      assert(std::abs(estimator.rpm() - end) < end * .025f);
    }
  }

  // Two identical antenna lobes produce equally credible half/full-period
  // peaks. A settled calibrated prior resolves this ambiguity.
  RssiRpmEstimator lobes;
  for (uint32_t t = 0; t < 2000000; t += 1000) {
    const int rssi = std::lround(-64 + 10 * std::sin(t * 2 * 3.141592653589793 / 20000));
    lobes.add_sample(t, rssi);
    if (t % 10000 == 0) {
      lobes.set_drive_hint(1500.f / 6784, 1500, t / 1000);
      lobes.update(t / 1000);
    }
  }
  assert(std::abs(lobes.rpm() - 1500) < 20);
  assert(lobes.diagnostics().hint_used);

  // A bad prior has no permission to invent a period in flat RSSI.
  RssiRpmEstimator flat;
  for (uint32_t t = 0; t < 1500000; t += 1000) {
    flat.add_sample(t, -60);
    flat.set_drive_hint(.4f, 2700, t / 1000);
    flat.update(t / 1000);
  }
  assert(flat.rpm() == 0);

  // Flash snapshots made by the previous firmware retain their RSSI/control
  // fields while the removed accelerometer RPM slot is discarded.
  RunDump::Header legacy{RunDump::magic, 1, 1, 1, 56, 0, 0, 0};
  uint8_t bytes[56]{};
  const float accel = 1999, rssi = 2001;
  std::memcpy(bytes + 20, &accel, sizeof(accel));
  std::memcpy(bytes + 24, &rssi, sizeof(rssi));
  RunDump::Record decoded;
  assert(RunDump::decode_record(bytes, legacy, decoded));
  assert(decoded.target_rpm == 0 && decoded.rssi_rpm == rssi);
}
