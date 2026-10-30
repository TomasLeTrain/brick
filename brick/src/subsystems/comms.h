#pragma once

#include "../common/packets.h"
#include "config.h"
#include "src/lib/NowPair.h"

#include <cstddef>
#include <cstdint>

class Comms {
public:
  Comms();
  ~Comms();

  bool send(TelemetryPacket *packet);
  bool begin();
  void update();
  bool connected();

  ThrottlePacket *get_throttle_packet() { return &current_throttle; }
  NowPair* get_link() { return peer_link; }

private:
  static void dataCb(const uint8_t *data, size_t length, void *arg);
  void on_data(const uint8_t *data, size_t length);
  void end();

  bool peerConnected = false;
  NowPair *peer_link = nullptr;

  ThrottlePacket current_throttle;
};
