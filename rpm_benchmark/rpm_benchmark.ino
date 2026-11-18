#include "../brick/src/subsystems/rssi_rpm_estimator.h"
#include "../brick/src/subsystems/rpm_heading_tracker.h"
#include "../common/rssi_simulation.h"
#include <esp_timer.h>

// This separate sketch runs without constructing Robot or initializing motors.
struct Timing {
  uint64_t total = 0;
  uint32_t maximum = 0, count = 0;
  void add(uint32_t us) {
    total += us;
    maximum = std::max(maximum, us);
    ++count;
  }
  float mean() const { return count ? float(total) / count : 0; }
};

RssiRpmEstimator estimator;
RpmHeadingTracker heading_tracker;

void runCase(float expected_rpm, bool dump, float end_rpm) {
  estimator.reset();
  heading_tracker.reset();
  RssiSimulation signal;
  Timing samples, updates, analyses;
  uint32_t last_update = 0, checks = 0, matches = 0;
  float squared_error = 0;
  uint32_t min_stack = uxTaskGetStackHighWaterMark(nullptr);
  char line[512];

  if (dump) {
    Serial.println("# dump_begin simulation_device");
    String header = RunDump::csv_header;
    header.trim();
    Serial.print(header);
    Serial.println(",true_rpm,true_phase_rad");
  }
  while (signal.time_us() < 2000000) {
    const float actual_rpm = RssiSimulation::ramp_rpm(
        signal.time_us(), expected_rpm, end_rpm, 2000000);
    const float target = signal.time_us() < 600000 ? expected_rpm : end_rpm;
    if (!signal.next(actual_rpm))
      continue;
    RunDump::Record row{};
    row.kind = 1;
    row.timestamp_us = row.received_us = signal.time_us();
    row.rssi_dbm = signal.rssi_dbm();
    uint64_t start = esp_timer_get_time();
    estimator.add_sample(row.timestamp_us, row.rssi_dbm);
    samples.add(esp_timer_get_time() - start);
    if (dump) {
      RunDump::csv_record(line, sizeof(line), row);
      line[std::strlen(line) - 1] = 0;
      Serial.printf("%s,%.1f,%.6f\n", line, actual_rpm, signal.phase_rad());
    }

    if (signal.time_us() - last_update < 10000)
      continue;
    last_update = signal.time_us();
    const uint32_t previous_analyses = estimator.analysis_count();
    start = esp_timer_get_time();
    estimator.set_drive_hint(target / 6784.08434167f, target, last_update / 1000);
    estimator.update(last_update / 1000);
    const auto &measurement = estimator.measurement();
    heading_tracker.update(last_update, estimator.rpm(),
        {measurement.timestamp_us, measurement.rpm, measurement.correlation});
    const uint32_t duration = esp_timer_get_time() - start;
    updates.add(duration);
    if (estimator.analysis_count() != previous_analyses)
      analyses.add(duration);
    min_stack = std::min(min_stack, uint32_t(uxTaskGetStackHighWaterMark(nullptr)));
    if (last_update > 700000) {
      ++checks;
      const float error = estimator.rpm() - actual_rpm;
      squared_error += error * error;
      if (std::abs(error) < actual_rpm * 0.025f)
        ++matches;
    }
    if (dump) {
      row.kind = 2;
      const auto &heading = heading_tracker.state();
      row.heading_rad = heading.heading_rad;
      row.heading_error_rad = heading.error_rad;
      row.rpm_age_ms = heading.measurement_age_ms;
      row.target_rpm = target;
      row.throttle = target / 6784.08434167f;
      const auto &d = estimator.diagnostics();
      row.fast_rpm = d.fast_rpm;
      row.long_rpm = d.long_rpm;
      row.correlation = d.correlation;
      row.analysis_timestamp_us = d.timestamp_us;
      row.comparison_samples = d.comparison_samples;
      row.min_lag = d.min_lag;
      row.max_lag = d.max_lag;
      row.tracking_mode = d.mode | (static_cast<uint8_t>(d.status) << 2);
      row.connected = 0x80;
      row.hint_used = uint8_t(d.hint_used) | (uint8_t(heading.status) << 1);
      row.rssi_rpm = estimator.rpm();
      row.control_rpm = heading.rpm;
      row.rssi_hz = estimator.sample_rate_hz();
      row.estimator_us = duration;
      RunDump::csv_record(line, sizeof(line), row);
      line[std::strlen(line) - 1] = 0;
      Serial.printf("%s,%.1f,%.6f\n", line, actual_rpm, signal.phase_rad());
    }
    // Keep the watchdog serviced. This wait is outside timed estimator calls.
    delay(1);
  }
  if (dump)
    Serial.println("# dump_end");
  const float load_percent =
      (samples.total + updates.total) * 100.0f / signal.time_us();
  Serial.printf("# bench expected_rpm=%.0f final_rpm=%.1f matches=%lu/%lu "
                "rmse=%.2f add_mean_us=%.2f add_max_us=%lu "
                "update_mean_us=%.2f update_max_us=%lu "
                "analysis_mean_us=%.2f analysis_max_us=%lu analyses=%lu "
                "estimated_cpu_pct=%.2f stack_free_bytes=%lu heap_free_bytes=%lu\n",
                expected_rpm, estimator.rpm(),
                static_cast<unsigned long>(matches), static_cast<unsigned long>(checks),
                std::sqrt(squared_error / std::max(uint32_t(1), checks)), samples.mean(),
                static_cast<unsigned long>(samples.maximum), updates.mean(),
                static_cast<unsigned long>(updates.maximum), analyses.mean(),
                static_cast<unsigned long>(analyses.maximum),
                static_cast<unsigned long>(analyses.count), load_percent,
                static_cast<unsigned long>(min_stack),
                static_cast<unsigned long>(ESP.getFreeHeap()));
}

void runHeadingCase() {
  heading_tracker.reset();
  RpmHeadingTracker dead;
  Timing updates;
  uint32_t checks = 0;
  float reference = 0, sum_error = 0, sum_dead = 0, peak = 0;
  const auto speed = [](int64_t t) {
    return 1200.f + 1800.f * std::max(0.f, std::min(1.f, (t - 1000000) / 2000000.f));
  };
  for (uint32_t now = 0; now <= 6000000; now += 10000) {
    const float measured = speed(int64_t(now) - 100000);
    const RpmHeadingTracker::Observation m{now - 100000, measured, .95f};
    const uint64_t start = esp_timer_get_time();
    heading_tracker.update(now, measured, m);
    updates.add(esp_timer_get_time() - start);
    dead.update(now, measured, m, false);
    const double t = now * 1e-6, ramp = std::max(0., std::min(2., t - 1));
    const float truth = RpmHeadingTracker::wrap(.10471975511965977 *
        (1200 * std::min(t, 1.) + 1200 * ramp + 450 * ramp * ramp + 3000 * std::max(0., t - 3)));
    const auto &heading = heading_tracker.state();
    if (now == 500000) reference = RpmHeadingTracker::wrap(heading.heading_rad - truth);
    if (now >= 1000000) peak = std::max(peak, std::abs(RpmHeadingTracker::wrap(heading.heading_rad - truth - reference)));
    if (now >= 4000000) {
      const float error = RpmHeadingTracker::wrap(heading.heading_rad - truth - reference);
      const float old_error = RpmHeadingTracker::wrap(dead.state().heading_rad - truth - reference);
      sum_error += error * error; sum_dead += old_error * old_error; ++checks;
    }
    delay(1);
  }
  Serial.printf("# heading_bench rpm_delay_ms=100 settled_rmse_deg=%.2f dead_rmse_deg=%.2f "
                "transient_peak_deg=%.2f update_mean_us=%.2f update_max_us=%lu "
                "estimated_cpu_pct=%.2f heap_free_bytes=%lu\n",
                std::sqrt(sum_error / checks) * 57.2957795f,
                std::sqrt(sum_dead / checks) * 57.2957795f, peak * 57.2957795f,
                updates.mean(), static_cast<unsigned long>(updates.maximum),
                updates.total * 100.f / 6000000,
                static_cast<unsigned long>(ESP.getFreeHeap()));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("# CPU MHz=%lu estimator_bytes=%u heading_tracker_bytes=%u\n",
                static_cast<unsigned long>(ESP.getCpuFreqMHz()),
                unsigned(sizeof(RssiRpmEstimator)), unsigned(sizeof(RpmHeadingTracker)));
  Serial.println("# b benchmarks RPM+heading; p benchmarks delayed-RPM heading correction; d dumps 3000 RPM; r dumps a ramp");
}

void loop() {
  if (Serial.available()) {
    const char command = Serial.read();
    if (command == 'b') {
      for (float rpm : {400.f, 600.f, 1111.f, 2275.f, 3000.f})
        runCase(rpm, false, rpm);
    } else if (command == 'p') {
      runHeadingCase();
    } else if (command == 'd') {
      runCase(3000, true, 3000);
    } else if (command == 'r') {
      runCase(1200, true, 3000);
    }
  }
  delay(10);
}
