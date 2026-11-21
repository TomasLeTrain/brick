#pragma once
#include "../common/companion.h"
#include "../common/radio_config.h"
#include "NowPair.h"
#include <Arduino.h>
#include <cstdio>

// One bounded serial line at a time. The sketch arbitrates with DumpReceiver so
// telemetry/status can never split a CSV row, even with USB backpressure.
class SenderConsole {
public:
  CompanionControl companion;
  bool printing() const { return position < size; }
  void receive(const TelemetryPacket &packet) {
    latest = packet;
    have_telemetry = true;
    last_received_ms = millis();
  }
  const char *readLine(char c) { return input.feed(c); }
  void status(const char *message) {
    std::snprintf(pending_status, sizeof(pending_status), "# %s\n", message);
  }
  void update(NowPair &link, bool dump_busy) {
    if (!printing() && !dump_busy) {
      position = size = 0;
      if (pending_status[0]) {
        size = std::snprintf(output, sizeof(output), "%s", pending_status);
        pending_status[0] = 0;
      } else if (millis() - last_report_ms >= 100) {
        last_report_ms = millis();
        const auto tx = link.txStats();
        const auto &p = latest;
        const int written = std::snprintf(
            output, sizeof(output),
            "@T {\"v\":1,\"link\":%u,\"have_telemetry\":%u,\"rx_age_ms\":%lu,"
            "\"rpm\":%.2f,\"target_rpm\":%.2f,\"throttle\":%.4f,\"rssi_hz\":%."
            "2f,"
            "\"correlation\":%.3f,\"heading_rad\":%.4f,\"heading_error_rad\":%."
            "4f,\"rpm_age_ms\":%.2f,"
            "\"rssi_dbm\":%d,\"control_age_ms\":%u,\"control_sequence\":%lu,"
            "\"connection_state\":%u,"
            "\"rpm_status\":%u,\"heading_status\":%u,\"inverted\":%u,"
            "\"companion_active\":%u,\"max_throttle\":%u,"
            "\"controller_connected\":%u,\"spin_enabled\":%u,\"rx_dropped\":%"
            "lu,\"rssi_dropped\":%lu,"
            "\"robot_tx_failures\":%lu,\"tx_attempts\":%lu,\"tx_completed\":%"
            "lu,\"tx_failures\":%lu,"
            "\"tx_max_us\":%lu,\"pc_active\":%u,\"sender_inverted\":%u,"
            "\"channel\":%u}\n",
            unsigned(link.connected()), unsigned(have_telemetry),
            static_cast<unsigned long>(
                have_telemetry ? millis() - last_received_ms : UINT32_MAX),
            p.rpm, p.target_rpm, p.throttle, p.rssi_hz, p.correlation,
            p.heading_rad, p.heading_error_rad, p.rpm_age_ms, int(p.rssi_dbm),
            unsigned(p.control_age_ms),
            static_cast<unsigned long>(p.control_sequence),
            unsigned(p.connection_state), unsigned(p.rpm_status),
            unsigned(p.heading_status), unsigned(p.inverted),
            unsigned(p.companion_active), unsigned(p.max_throttle),
            unsigned(p.controller_connected),
            unsigned(p.spin_enabled), static_cast<unsigned long>(p.rx_dropped),
            static_cast<unsigned long>(p.rssi_dropped),
            static_cast<unsigned long>(p.tx_failures),
            static_cast<unsigned long>(tx.attempts),
            static_cast<unsigned long>(tx.completed),
            static_cast<unsigned long>(tx.failures),
            static_cast<unsigned long>(tx.max_completion_us),
            unsigned(companion.owns_heading(millis())),
            unsigned(companion.is_inverted()), unsigned(RADIO_CHANNEL));
        if (written > 0 && size_t(written) < sizeof(output))
          size = written;
      }
    }
    const size_t bytes = std::min(
        size - position, size_t(std::max(0, Serial.availableForWrite())));
    if (bytes)
      position += Serial.write(
          reinterpret_cast<const uint8_t *>(output + position), bytes);
  }

private:
  SerialLines input;
  TelemetryPacket latest{};
  bool have_telemetry = false;
  uint32_t last_received_ms = 0, last_report_ms = 0;
  char output[1024]{}, pending_status[128]{};
  size_t size = 0, position = 0;
};
