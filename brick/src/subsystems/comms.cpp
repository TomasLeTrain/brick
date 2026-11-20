#include "comms.h"
#include <cmath>

const char *Comms::heading_command(char command, void *arg) {
  auto &tracker = static_cast<Comms *>(arg)->heading_tracker;
  using Mode = RpmHeadingTracker::CorrectionMode;
  switch (command) {
  case 'a':
    tracker.set_correction_mode(Mode::AccelerationOnly);
    return "heading acceleration-only; translation uses dead reckoning";
  case 't':
    tracker.set_correction_mode(Mode::Full);
    return "heading correction enabled, including translation";
  case 'n':
    tracker.set_correction_mode(Mode::Disabled);
    return "heading correction disabled; plain dead reckoning";
  default:
    return nullptr;
  }
}

void Comms::connectionCb(bool connected, void *arg) {
  if (!connected) {
    auto &self = *static_cast<Comms *>(arg);
    self.current_throttle = {};
    self.control_freshness.reset();
    self.control_live.store(false);
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
  }
}

void Comms::controlCb(const uint8_t *data, size_t length, uint32_t received_ms,
                      void *arg) {
  auto &self = *static_cast<Comms *>(arg);
  ThrottlePacket packet;
  if (!packet.deserialize(data, length) ||
      !self.control_freshness.accept(packet, received_ms, millis()))
    return;
  self.current_throttle = packet;
  self.control_received_ms.store(received_ms);
  self.control_live.store(packet.connected && packet.swa);
  if (packet.connected) {
    if (self.switch_seen && self.last_swb && !packet.swb)
      self.recorder.request('s');
    self.last_swb = packet.swb;
    self.switch_seen = true;
  } else
    self.switch_seen = false;
}

void Comms::rssiCb(int rssi_dbm, uint32_t timestamp_us, uint32_t callback_us,
                   void *arg) {
  auto &self = *static_cast<Comms *>(arg);
  self.latest_rssi = rssi_dbm;
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
  using Mode = RpmHeadingTracker::CorrectionMode;
  heading_tracker.set_correction_mode(!RPM_HEADING_COMPENSATION ? Mode::Disabled
                                      : RPM_HEADING_TRANSLATION_COMPENSATION
                                          ? Mode::Full
                                          : Mode::AccelerationOnly);
  recorder.on_control_command(heading_command, this);
  recorder.begin();
  peer_link.onConnectionChange(connectionCb, this);
  peer_link.onData(dataCb, this);
  peer_link.onRealtime(controlCb, this);
  peer_link.expectPeer(SENDER_MAC);
  const esp_now_rate_config_t rate = {WIFI_PHY_MODE_11G, WIFI_PHY_RATE_6M,
                                      false, false};
  peer_link.setRate(&rate);
  peer_link.onRssiTimed(rssiCb, this);
  return peer_link.begin();
}

void Comms::update() {
  peer_link.update();

  const uint32_t now_ms = millis();
  if (!control_freshness.fresh(now_ms)) {
    control_live.store(false);
    current_throttle.connected = false;
    current_throttle.companion_active = false;
    switch_seen = false;
  }
}

void Comms::update_rpm(float throttle, float steady_rpm, bool translating) {
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
      heading_observation = {
          now - uint32_t(std::lround(age * RPM_HEADING_DELAY_SCALE)),
          measurement.rpm, measurement.correlation};
    }
  }
  heading_tracker.set_translation(translating);
  heading_tracker.update(now, rssi_estimator.rpm(), heading_observation);
  estimator_us = micros() - started_us;
}
