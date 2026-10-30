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
  // all represent a range between 0 - 1000
  // when serializing they are packed to fit inside 8 bytes
  uint16_t left_lr;
  uint16_t left_ud;

  uint16_t right_lr;
  uint16_t right_ud;

  uint16_t knob_left;
  uint16_t knob_right;

  size_t serialize(uint8_t *buffer);
  bool deserialize(const uint8_t *data, size_t length);
  static size_t expected_packet_size() { return 8; }
};

inline size_t ThrottlePacket::serialize(uint8_t *buffer) {
  uint64_t result = 0;

  uint16_t mask = (1 << 10) - 1;

  // assumes all values are not invalid, caps them anyway
  result |= uint64_t(left_lr & mask);
  result |= uint64_t(left_ud & mask) << 10;
  result |= uint64_t(right_lr & mask) << 20;
  result |= uint64_t(right_ud & mask) << 30;
  result |= uint64_t(knob_left & mask) << 40;
  result |= uint64_t(knob_right & mask) << 50;

  for (int i = 0; i < 8; i++) {
    buffer[i] = (result >> (8 * i)) & 0xff;
  }

  return 8;
}

inline bool ThrottlePacket::deserialize(const uint8_t *data, size_t length) {
  if (length != expected_packet_size()) {
    return false;
  }

  uint64_t result = 0;

  for (int i = 0; i < 8; i++) {
    result |= uint64_t(data[i]) << uint64_t(8 * i);
  }

  uint16_t mask = (1 << 10) - 1;

  left_lr = result & mask;
  left_ud = (result >> 10) & mask;
  right_lr = (result >> 20) & mask;
  right_ud = (result >> 30) & mask;
  knob_left = (result >> 40) & mask;
  knob_right = (result >> 50) & mask;

  return true;
}
