#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

// Explicit application protocol. Both ESPs must be updated together; old
// acceleration telemetry and unversioned controls are deliberately rejected.
namespace PacketWire {
inline void u32(uint8_t *p, uint32_t n) {
  for (int i = 0; i < 4; ++i)
    p[i] = n >> (8 * i);
}
inline uint32_t u32(const uint8_t *p) {
  uint32_t n = 0;
  for (int i = 0; i < 4; ++i)
    n |= uint32_t(p[i]) << (8 * i);
  return n;
}
inline void f32(uint8_t *p, float v) {
  static_assert(sizeof(float) == 4, "32-bit IEEE float required");
  uint32_t n;
  std::memcpy(&n, &v, 4);
  u32(p, n);
}
inline float f32(const uint8_t *p) {
  uint32_t n = u32(p);
  float v;
  std::memcpy(&v, &n, 4);
  return v;
}
inline void header(uint8_t *p, uint8_t type, uint8_t version = 1) {
  p[0] = 'B';
  p[1] = 'R';
  p[2] = version;
  p[3] = type;
}
inline bool header(const uint8_t *p, size_t n, uint8_t type, size_t expected,
                   uint8_t version = 1) {
  return p && n == expected && p[0] == 'B' && p[1] == 'R' && p[2] == version &&
         p[3] == type;
}
} // namespace PacketWire

struct TelemetryPacket {
  uint32_t micros_timestamp = 0;
  float rpm = 0, target_rpm = 0, throttle = 0, rssi_hz = 0;
  float correlation = 0, heading_rad = 0, heading_error_rad = 0, rpm_age_ms = 0;
  uint32_t rx_dropped = 0, rssi_dropped = 0, tx_failures = 0;
  uint8_t connection_state = 0, rpm_status = 0, heading_status = 0;
  bool inverted = false, companion_active = false, controller_connected = false;
  bool spin_enabled = false;
  int16_t rssi_dbm = -128;
  uint16_t control_age_ms = UINT16_MAX;
  uint32_t control_sequence = 0;

  static constexpr size_t expected_packet_size() { return 64; }
  size_t serialize(uint8_t *p) const {
    PacketWire::header(p, 2);
    PacketWire::u32(p + 4, micros_timestamp);
    const float values[] = {
        rpm,         target_rpm,  throttle,          rssi_hz,
        correlation, heading_rad, heading_error_rad, rpm_age_ms};
    for (int i = 0; i < 8; ++i)
      PacketWire::f32(p + 8 + 4 * i, std::isfinite(values[i]) ? values[i] : 0);
    PacketWire::u32(p + 40, rx_dropped);
    PacketWire::u32(p + 44, rssi_dropped);
    PacketWire::u32(p + 48, tx_failures);
    p[52] = connection_state;
    p[53] = rpm_status;
    p[54] = heading_status;
    p[55] = uint8_t(inverted) | (uint8_t(companion_active) << 1) |
            (uint8_t(controller_connected) << 2) | (uint8_t(spin_enabled) << 3);
    p[56] = uint16_t(rssi_dbm);
    p[57] = uint16_t(rssi_dbm) >> 8;
    p[58] = control_age_ms;
    p[59] = control_age_ms >> 8;
    PacketWire::u32(p + 60, control_sequence);
    return expected_packet_size();
  }
  bool deserialize(const uint8_t *p, size_t n) {
    if (!PacketWire::header(p, n, 2, expected_packet_size()) || (p[55] & 0xf0))
      return false;
    float values[8];
    for (int i = 0; i < 8; ++i) {
      values[i] = PacketWire::f32(p + 8 + 4 * i);
      if (!std::isfinite(values[i]))
        return false;
    }
    micros_timestamp = PacketWire::u32(p + 4);
    rpm = values[0];
    target_rpm = values[1];
    throttle = values[2];
    rssi_hz = values[3];
    correlation = values[4];
    heading_rad = values[5];
    heading_error_rad = values[6];
    rpm_age_ms = values[7];
    rx_dropped = PacketWire::u32(p + 40);
    rssi_dropped = PacketWire::u32(p + 44);
    tx_failures = PacketWire::u32(p + 48);
    connection_state = p[52];
    rpm_status = p[53];
    heading_status = p[54];
    inverted = p[55] & 1;
    companion_active = p[55] & 2;
    controller_connected = p[55] & 4;
    spin_enabled = p[55] & 8;
    const uint16_t signed_rssi = uint16_t(p[56]) | (uint16_t(p[57]) << 8);
    rssi_dbm = signed_rssi < 32768 ? signed_rssi : int32_t(signed_rssi) - 65536;
    control_age_ms = uint16_t(p[58]) | (uint16_t(p[59]) << 8);
    control_sequence = PacketWire::u32(p + 60);
    return true;
  }
};

struct ThrottlePacket {
  uint16_t left_lr = 500, left_ud = 0, right_lr = 500, right_ud = 500;
  uint16_t knob_left = 0, knob_right = 0;
  bool connected = false, swa = false, swb = false;
  bool inverted = false, companion_active = false;
  uint32_t session = 0, sequence = 0;
  uint8_t orientation_request =
      0; // Seven-bit action revision, repeated over RF.
  static constexpr size_t expected_packet_size() { return 21; }
  size_t serialize(uint8_t *p) const {
    PacketWire::header(p, 1, 2);
    p[4] = uint8_t(companion_active) | ((orientation_request & 127) << 1);
    PacketWire::u32(p + 5, session);
    PacketWire::u32(p + 9, sequence);
    const uint16_t values[] = {left_lr,  left_ud,   right_lr,
                               right_ud, knob_left, knob_right};
    uint64_t packed = 0;
    for (int i = 0; i < 6; ++i)
      packed |= uint64_t(std::min(values[i], uint16_t(1000))) << (10 * i);
    packed |= uint64_t(connected) << 60;
    packed |= uint64_t(swa) << 61;
    packed |= uint64_t(swb) << 62;
    packed |= uint64_t(inverted) << 63;
    for (int i = 0; i < 8; ++i)
      p[13 + i] = packed >> (8 * i);
    return expected_packet_size();
  }
  bool deserialize(const uint8_t *p, size_t n) {
    if (!PacketWire::header(p, n, 1, expected_packet_size(), 2))
      return false;
    uint64_t packed = 0;
    for (int i = 0; i < 8; ++i)
      packed |= uint64_t(p[13 + i]) << (8 * i);
    uint16_t values[6];
    for (int i = 0; i < 6; ++i) {
      values[i] = (packed >> (10 * i)) & 1023;
      if (values[i] > 1000)
        return false;
    }
    left_lr = values[0];
    left_ud = values[1];
    right_lr = values[2];
    right_ud = values[3];
    knob_left = values[4];
    knob_right = values[5];
    connected = (packed >> 60) & 1;
    swa = (packed >> 61) & 1;
    swb = (packed >> 62) & 1;
    inverted = packed >> 63;
    companion_active = p[4] & 1;
    orientation_request = p[4] >> 1;
    session = PacketWire::u32(p + 5);
    sequence = PacketWire::u32(p + 9);
    return session != 0;
  }
};

// A duplicate or late control must not refresh the motor-enable watchdog.
class ControlFreshness {
public:
  static constexpr uint32_t timeout_ms = 100;
  bool accept(const ThrottlePacket &p, uint32_t received_ms, uint32_t now_ms) {
    if (!p.session || uint32_t(now_ms - received_ms) >= timeout_ms)
      return false;
    if (seen && p.session == session && int32_t(p.sequence - sequence) <= 0)
      return false;
    if (seen && p.session != session && uint32_t(now_ms - last_ms) < timeout_ms)
      return false;
    seen = true;
    session = p.session;
    sequence = p.sequence;
    last_ms = received_ms;
    return true;
  }
  bool fresh(uint32_t now_ms) const {
    return seen && uint32_t(now_ms - last_ms) < timeout_ms;
  }
  uint16_t age_ms(uint32_t now_ms) const {
    return seen ? std::min(uint32_t(now_ms - last_ms), uint32_t(UINT16_MAX))
                : UINT16_MAX;
  }
  void reset() { seen = false; }

private:
  bool seen = false;
  uint32_t session = 0, sequence = 0, last_ms = 0;
};
