#pragma once

#include "../common/packets.h"
#include "../../config.h"
#include "rssi_rpm_estimator.h"
#include "../lib/NowPair.h"

class Comms {
public:
  bool send(TelemetryPacket *packet);
  bool begin();
  void update();
  bool connected() const { return peer_link.connected(); }

  ThrottlePacket *get_throttle_packet() { return &current_throttle; }
  const RssiRpmEstimator &get_rssi_estimator() const { return rssi_estimator; }

private:
  static void connectionCb(bool connected, void *arg);
  static void dataCb(const uint8_t *data, size_t length, int rssi_dbm,
                     uint32_t timestamp_us, void *arg);
  static void rssiCb(int rssi_dbm, uint32_t timestamp_us, void *arg);

  NowPair peer_link{NowPair::Role::Slave, 6, HEARTBEAT_MS, TIMEOUT_MS};
  RssiRpmEstimator rssi_estimator;
  ThrottlePacket current_throttle{};
  uint32_t last_throttle_ms = 0;
};
