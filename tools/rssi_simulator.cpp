#include "../brick/src/subsystems/rssi_rpm_estimator.h"
#include "../common/rssi_simulation.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

// stdout uses the same CSV columns as device captures plus true_rpm.
int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: rssi_simulator RPM SECONDS PACKETS_PER_SECOND > simulation.csv\n";
    return 2;
  }
  try {
    const float rpm = std::stof(argv[1]);
    const double seconds = std::stod(argv[2]);
    const unsigned packet_rate = std::stoul(argv[3]);
    if (!std::isfinite(rpm) || rpm < 0 || rpm > 10000 ||
        !std::isfinite(seconds) || seconds <= 0 || seconds > 600 ||
        packet_rate < 100 || packet_rate > 10000)
      throw std::invalid_argument("arguments out of range");

    RssiRpmEstimator estimator;
    RssiSimulation signal;
    std::cout << "# dump_begin simulation\n";
    std::string header = RunDump::csv_header;
    header.pop_back();
    std::cout << header << ",true_rpm\n";
    uint32_t last_update = 0;
    while (signal.time_us() < seconds * 1000000.0) {
      if (!signal.next(rpm, packet_rate))
        continue;
      RunDump::Record row{};
      row.kind = 1;
      row.timestamp_us = row.received_us = signal.time_us();
      row.rssi_dbm = signal.rssi_dbm();
      estimator.add_sample(row.timestamp_us, row.rssi_dbm);
      char line[256];
      RunDump::csv_record(line, sizeof(line), row);
      std::string csv = line;
      csv.pop_back();
      std::cout << csv << ',' << rpm << '\n';
      if (signal.time_us() - last_update < 10000)
        continue;
      last_update = signal.time_us();
      const auto start = std::chrono::steady_clock::now();
      estimator.update(last_update / 1000);
      row.estimator_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - start).count();
      row.kind = 2;
      const float omega = rpm * 0.104719755f;
      row.ax_g = omega * omega * 4.846f / 981.0f;
      row.az_g = 1;
      row.accel_rpm = rpm;
      row.rssi_rpm = row.control_rpm = estimator.rpm();
      row.rssi_hz = estimator.sample_rate_hz();
      RunDump::csv_record(line, sizeof(line), row);
      csv = line;
      csv.pop_back();
      std::cout << csv << ',' << rpm << '\n';
    }
    std::cout << "# dump_end\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
