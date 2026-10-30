#pragma once

#include <cstddef>
#include <cstdint>

struct TelemetryPacket {
  uint16_t a_x;
  uint16_t a_y;
  uint16_t a_z;

  bool deserialize(const uint8_t *data, size_t length);
  size_t serialize(uint8_t *buffer);
  static size_t expected_packet_size() { return 6; }
};

inline size_t TelemetryPacket::serialize(uint8_t *buffer) {
  buffer[0] = a_x & 0xff;
  buffer[1] = (a_x >> 8) & 0xff;
  buffer[2] = a_y & 0xff;
  buffer[3] = (a_y >> 8) & 0xff;
  buffer[4] = a_z & 0xff;
  buffer[5] = (a_z >> 8) & 0xff;
  return 6;
}

inline bool TelemetryPacket::deserialize(const uint8_t *data, size_t length) {
  if (length != expected_packet_size()) {
    return false;
  }

  a_x = data[0] | (data[1] << 8);
  a_y = data[2] | (data[3] << 8);
  a_z = data[4] | (data[5] << 8);

  return true;
}

struct ThrottlePacket {
  uint16_t throttle;

  size_t serialize(uint8_t *buffer);
  bool deserialize(const uint8_t *data, size_t length);
  static size_t expected_packet_size() { return 2; }
};

inline size_t ThrottlePacket::serialize(uint8_t *buffer) {
  buffer[0] = throttle & 0xff;
  buffer[1] = (throttle >> 8) & 0xff;
  return 2;
}

inline bool ThrottlePacket::deserialize(const uint8_t *data, size_t length) {
  if (length != expected_packet_size()) {
    return false;
  }

  this->throttle = data[0] | (data[1] << 8);
  return true;
}
