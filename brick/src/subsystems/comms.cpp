#include "config.h"
#include "src/lib/NowPair.h"

struct TelemetryPacket {
  uint16_t a_x;
  uint16_t a_y;
  uint16_t a_z;

  // TODO: bit endianess?
  size_t serialize(uint8_t *buffer) {
    buffer[0] = a_x & 0xff;
    buffer[1] = (a_x >> 8) & 0xff;
    buffer[2] = a_y & 0xff;
    buffer[3] = (a_y >> 8) & 0xff;
    buffer[4] = a_z & 0xff;
    buffer[5] = (a_z >> 8) & 0xff;
    return 6;
  }
};

class Comms {
  Comms() {}
  ~Comms() { end(); }

  void end() {
    if (peer_link) {
      delete peer_link;
    }
  }

  static void dataCb(const uint8_t *data, size_t length, void *arg) {
    if (data == nullptr || length < 0)
      return;
    auto *self = static_cast<Comms *>(arg);
    self->on_data(data, length);
  }

  void on_data(const uint8_t *data, size_t length) {
    // TODO: deserialize message and process appropriately
  }

  bool send(TelemetryPacket *packet) {
    // max packet is 228, anything getting serialized should fit below that
    static uint8_t data_buffer[228];
    size_t n = packet->serialize(data_buffer);
    return peer_link->sendData(data_buffer, n);
  }

  bool begin() {
    peer_link = new NowPair(NowPair::Role::Slave, 6, NETWORK_ID, HEARTBEAT_MS,
                            TIMEOUT_MS);

    peer_link->onData(dataCb, this);

    if (!peer_link->begin()) {
      end();
      return false;
    }
    return true;
  }

  void update() { peer_link->update(); }

  bool connected() { peer_link->connected(); }

private:
  bool peerConnected = false;
  NowPair *peer_link = nullptr;
};
