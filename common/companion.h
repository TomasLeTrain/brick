#pragma once
#include "packets.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

// USB serial:
// @C,1,session,sequence,turn(-1000..1000),orientation(-1/0/1),enabled
// Orientation is absolute and latched. Send 0/1 once per button press; normal
// heartbeats send -1. Repeated RF controls retain the same acknowledgement ID.
// Heading ownership is a short lease; the computer cannot arm or set throttle.
class CompanionControl {
public:
  static constexpr uint32_t lease_ms = 150;
  bool receive(const char *line, uint32_t now_ms) {
    if (std::strncmp(line, "@C,", 3) != 0)
      return false;
    const char *cursor = line + 3;
    int64_t fields[6];
    for (int i = 0; i < 6; ++i) {
      const bool negative = *cursor == '-';
      if (negative)
        ++cursor;
      if (*cursor < '0' || *cursor > '9')
        return false;
      uint64_t value = 0;
      while (*cursor >= '0' && *cursor <= '9') {
        value = value * 10 + (*cursor++ - '0');
        if (value > UINT32_MAX)
          return false;
      }
      fields[i] = negative ? -int64_t(value) : int64_t(value);
      if ((i < 5 && *cursor++ != ',') || (i == 5 && *cursor != '\0'))
        return false;
    }
    if (fields[0] != 1 || fields[1] < 1 || fields[2] < 0 || fields[3] < -1000 ||
        fields[3] > 1000 || fields[4] < -1 || fields[4] > 1 || fields[5] < 0 ||
        fields[5] > 1)
      return false;
    const uint32_t session = fields[1], sequence = fields[2];
    const int turn = fields[3], orientation = fields[4];
    const bool enabled = fields[5];
    if (seen && session == current_session &&
        int32_t(sequence - current_sequence) <= 0)
      return false;
    if (seen && session != current_session &&
        uint32_t(now_ms - last_ms) < lease_ms)
      return false;
    seen = true;
    current_session = session;
    current_sequence = sequence;
    last_ms = now_ms;
    active = enabled;
    turn_milli = turn;
    if (orientation >= 0) {
      inverted = orientation;
      orientation_request = (orientation_request + 1) & 127;
    }
    return true;
  }
  bool owns_heading(uint32_t now_ms) const {
    return seen && active && uint32_t(now_ms - last_ms) < lease_ms;
  }
  void apply(ThrottlePacket &packet, uint32_t now_ms) const {
    packet.inverted = inverted;
    packet.orientation_request = orientation_request;
    packet.companion_active = packet.connected && owns_heading(now_ms);
    if (packet.companion_active)
      packet.left_lr = (turn_milli + 1000) / 2;
  }
  bool is_inverted() const { return inverted; }

private:
  bool seen = false, active = false, inverted = false;
  uint8_t orientation_request = 0;
  int turn_milli = 0;
  uint32_t current_session = 0, current_sequence = 0, last_ms = 0;
};

// Bounded, newline-framed parsing. Overflow and partial lines cannot execute a
// command. No Arduino String allocations in the radio service loop.
class SerialLines {
public:
  const char *feed(char c) {
    if (c == '\r' || c == '\n') {
      const bool ready = size && !overflow;
      buffer[size] = '\0';
      size = 0;
      overflow = false;
      return ready ? buffer : nullptr;
    }
    if (static_cast<unsigned char>(c) < 32 ||
        static_cast<unsigned char>(c) > 126 || size + 1 >= sizeof(buffer))
      overflow = true;
    else if (!overflow)
      buffer[size++] = c;
    return nullptr;
  }

private:
  char buffer[96]{};
  size_t size = 0;
  bool overflow = false;
};
