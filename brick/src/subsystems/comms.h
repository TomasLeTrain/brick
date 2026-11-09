#pragma once

#include "../common/packets.h"
#include "config.h"
#include "src/lib/NowPair.h"

#include <cstddef>
#include <cstdint>

#define RSSI_QUEUE_SIZE 1000

class Comms {
public:
  Comms();
  ~Comms();

  bool send(TelemetryPacket *packet);
  bool begin();
  void update();
  bool connected();

  ThrottlePacket *get_throttle_packet() { return &current_throttle; }
  NowPair *get_link() { return peer_link; }

  void dump_rssi();
  void print_dump();

private:
  static void dataCb(const uint8_t *data, size_t length, int rssiDbm,
                     int rssiTimestamp, void *arg);
  void on_data(const uint8_t *data, size_t length, int rssiDbm,
               int rssiTimestamp);
  void end();

  void new_rssi_entry(int rssiDbm, int rssiTimestamp);

  bool peerConnected = false;
  NowPair *peer_link = nullptr;

  ThrottlePacket current_throttle;

  int rssi[RSSI_QUEUE_SIZE];
  int rssi_timestamps[RSSI_QUEUE_SIZE];
  int rssi_idx = 0;
  int rssi_capacity = 0;

  bool dumping_rssi = false;
};
