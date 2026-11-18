#include "comms.h"
#include <cmath>

namespace {
constexpr uint32_t throttle_timeout_ms = 100;
}

void Comms::connectionCb(bool connected, void *arg) {
  if (!connected) {
    auto &self = *static_cast<Comms *>(arg);
    self.current_throttle = {};
    self.rssi_estimator.reset();
    self.radio_clock.reset();
    self.heading_observed = false;
    self.heading_observation = {};
    self.switch_seen = false;
  }
}

void Comms::dataCb(const uint8_t *data, size_t length, int, uint32_t,
                   void *arg) {
  auto &self = *static_cast<Comms *>(arg);
  if (RunDump::is_message(data, length)) {
    self.recorder.on_message(data, length);
  } else if (self.current_throttle.deserialize(data, length)) {
    self.last_throttle_ms = millis();
    if (self.current_throttle.connected) {
      if (self.switch_seen && self.last_swb && !self.current_throttle.swb)
        self.recorder.request('s');
      self.last_swb = self.current_throttle.swb;
      self.switch_seen = true;
    } else {
      self.switch_seen = false;
    }
  }
}

void Comms::rssiCb(int rssi_dbm, uint32_t timestamp_us, uint32_t callback_us, void *arg) {
  auto &self = *static_cast<Comms *>(arg);
  if (self.connected()) {
    self.recorder.add_rssi(rssi_dbm, timestamp_us, callback_us);
    self.rssi_estimator.add_sample(timestamp_us, rssi_dbm);
    self.radio_clock.observe(timestamp_us, callback_us);
  }
}

bool Comms::send(TelemetryPacket *packet) {
  // NowPair transmits at most one frame per update, including heartbeats.
  // Per-loop telemetry can fill its shared data queue and claim each freed
  // slot before the recorder ever gets to queue its first export packet.
  const uint32_t now_ms = millis();
  if (recorder.radio_pending() || now_ms - last_telemetry_ms < 50)
    return false;
  uint8_t data_buffer[NowPair::MAX_PAYLOAD];
  const size_t length = packet->serialize(data_buffer);
  if (!peer_link.sendData(data_buffer, length))
    return false;
  last_telemetry_ms = now_ms;
  return true;
}

bool Comms::begin() {
  recorder.begin();
  peer_link.onConnectionChange(connectionCb, this);
  peer_link.onData(dataCb, this);
  peer_link.onRssiTimed(rssiCb, this);
  return peer_link.begin();
}

void Comms::update() {
  peer_link.update();

  const uint32_t now_ms = millis();
  if (now_ms - last_throttle_ms >= throttle_timeout_ms) {
    current_throttle.connected = false;
    switch_seen = false;
  }
}

void Comms::update_rpm(float throttle, float steady_rpm) {
  const uint32_t started_us = micros();
  rssi_estimator.set_drive_hint(throttle, steady_rpm, millis());
  rssi_estimator.update(millis());
  const uint32_t now = micros();
  const auto &measurement = rssi_estimator.measurement();
  if (radio_clock.available() && measurement.rpm > 0 &&
      (!heading_observed || measurement.timestamp_us != heading_radio_us)) {
    // Freeze the bridge/scale at receipt. Re-evaluating a cached observation
    // against a later now would manufacture fresh epochs when scale != 1.
    heading_observed = true;
    heading_radio_us = measurement.timestamp_us;
    heading_observation = {};
    const uint32_t local_time = radio_clock.local(measurement.timestamp_us);
    const int32_t age = int32_t(now - local_time);
    if (age >= 0 && age <= 250000) {
      heading_observation = {now - uint32_t(std::lround(age * RPM_HEADING_DELAY_SCALE)),
                             measurement.rpm, measurement.correlation};
    }
  }
  heading_tracker.update(now, rssi_estimator.rpm(), heading_observation, RPM_HEADING_COMPENSATION);
  estimator_us = micros() - started_us;
}
