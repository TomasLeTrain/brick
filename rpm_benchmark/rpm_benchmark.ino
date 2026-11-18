#include "../brick/src/subsystems/rssi_rpm_estimator.h"
#include "../brick/src/subsystems/rssi_phase_tracker.h"
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
RssiPhaseTracker phase_tracker;

void runCase(float expected_rpm, bool dump, float end_rpm) {
  estimator.reset();
  phase_tracker.reset();
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
    phase_tracker.add_sample(row.timestamp_us, row.received_us, row.rssi_dbm);
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
    phase_tracker.update(last_update, estimator.rpm());
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
      const auto &phase = phase_tracker.state();
      row.phase_rad = phase.phase_rad;
      row.phase_error_rad = phase.error_rad;
      row.phase_confidence = phase.confidence;
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
      row.hint_used = uint8_t(d.hint_used) | (uint8_t(phase.status) << 1);
      row.rssi_rpm = estimator.rpm();
      row.control_rpm = phase.rpm;
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

void runPhaseCase() {
  phase_tracker.reset();
  RssiSimulation signal;
  Timing adds, updates;
  uint32_t last_update = 0, checks = 0, locked = 0;
  float reference = 0, dead_phase = 0, phase_error2 = 0, dead_error2 = 0;
  bool anchored = false;
  constexpr float true_rpm = 2100, measured_rpm = .98f * true_rpm;
  while (signal.time_us() < 6000000) {
    if (!signal.next(true_rpm)) continue;
    uint64_t start = esp_timer_get_time();
    phase_tracker.add_sample(signal.time_us(), signal.time_us(), signal.rssi_dbm());
    adds.add(esp_timer_get_time() - start);
    const uint32_t now = signal.time_us();
    if (now - last_update < 10000) continue;
    if (last_update)
      dead_phase = RssiPhaseTracker::wrap(dead_phase + measured_rpm * .104719755f *
                                         float(now - last_update) * 1e-6f);
    last_update = now;
    start = esp_timer_get_time();
    phase_tracker.update(now, measured_rpm);
    updates.add(esp_timer_get_time() - start);
    const auto &phase = phase_tracker.state();
    if (phase.status == RssiPhaseTracker::Status::Tracking) {
      ++locked;
      if (!anchored) {
        reference = RssiPhaseTracker::wrap(phase.phase_rad - signal.phase_rad());
        anchored = true;
      }
    }
    if (anchored && now >= 2000000) {
      const float error = RssiPhaseTracker::wrap(phase.phase_rad - signal.phase_rad() - reference);
      const float dead_error = RssiPhaseTracker::wrap(dead_phase - signal.phase_rad() - reference);
      phase_error2 += error * error;
      dead_error2 += dead_error * dead_error;
      ++checks;
    }
    delay(1);
  }
  Serial.printf("# phase_bench true_rpm=2100 measured_rpm=2058 locked=%lu/%lu "
                "phase_rmse_deg=%.2f dead_rmse_deg=%.2f "
                "add_mean_us=%.2f update_mean_us=%.2f update_max_us=%lu "
                "estimated_cpu_pct=%.2f heap_free_bytes=%lu\n",
                static_cast<unsigned long>(locked), static_cast<unsigned long>(updates.count),
                checks ? std::sqrt(phase_error2 / checks) * 57.2957795f : NAN,
                checks ? std::sqrt(dead_error2 / checks) * 57.2957795f : NAN,
                adds.mean(), updates.mean(), static_cast<unsigned long>(updates.maximum),
                (adds.total + updates.total) * 100.f / signal.time_us(),
                static_cast<unsigned long>(ESP.getFreeHeap()));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("# CPU MHz=%lu estimator_bytes=%u phase_tracker_bytes=%u\n",
                static_cast<unsigned long>(ESP.getCpuFreqMHz()),
                unsigned(sizeof(RssiRpmEstimator)), unsigned(sizeof(RssiPhaseTracker)));
  Serial.println("# b benchmarks RPM+phase; p benchmarks biased-RPM phase tracking; d dumps 3000 RPM; r dumps a ramp");
}

void loop() {
  if (Serial.available()) {
    const char command = Serial.read();
    if (command == 'b') {
      for (float rpm : {400.f, 600.f, 1111.f, 2275.f, 3000.f})
        runCase(rpm, false, rpm);
    } else if (command == 'p') {
      runPhaseCase();
    } else if (command == 'd') {
      runCase(3000, true, 3000);
    } else if (command == 'r') {
      runCase(1200, true, 3000);
    }
  }
  delay(10);
}
