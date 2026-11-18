#pragma once

#include "../common/packets.h"
#include "../../config.h"
#include "rssi_rpm_estimator.h"
#include "run_recorder.h"
#include "../lib/NowPair.h"

class Comms {
public:
  bool send(TelemetryPacket *packet);
  void update_recorder(bool idle) { recorder.update(peer_link, idle); }
  bool begin();
  void update();
  void update_rpm(float throttle, float steady_rpm);
  bool connected() const { return peer_link.connected(); }

  ThrottlePacket *get_throttle_packet() { return &current_throttle; }
  const RssiRpmEstimator &get_rssi_estimator() const { return rssi_estimator; }
  RunRecorder &get_recorder() { return recorder; }
  uint32_t estimator_time_us() const { return estimator_us; }
  uint32_t dropped_rx() const { return peer_link.droppedRxPackets(); }
  uint32_t dropped_rssi() const { return peer_link.droppedRssiSamples(); }

private:
  static void connectionCb(bool connected, void *arg);
  static void dataCb(const uint8_t *data, size_t length, int rssi_dbm,
                     uint32_t timestamp_us, void *arg);
  static void rssiCb(int rssi_dbm, uint32_t timestamp_us, void *arg);

  NowPair peer_link{NowPair::Role::Slave, 6, HEARTBEAT_MS, TIMEOUT_MS};
  RssiRpmEstimator rssi_estimator;
  RunRecorder recorder;
  uint32_t estimator_us = 0;
  bool switch_seen = false, last_swb = false;
  ThrottlePacket current_throttle{};
  uint32_t last_throttle_ms = 0;
  uint32_t last_telemetry_ms = 0;
};
