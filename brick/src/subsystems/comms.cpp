#include "comms.h"

Comms::Comms() {}
Comms::~Comms() { end(); }

void Comms::end() {
  if (peer_link) {
    delete peer_link;
  }
}

void Comms::dataCb(const uint8_t *data, size_t length, void *arg) {
  if (data == nullptr || length < 0)
    return;
  auto *self = static_cast<Comms *>(arg);
  self->on_data(data, length);
}

void Comms::on_data(const uint8_t *data, size_t length) {
  // TODO: should definetely do something with res
  // maybe increase packet loss, or have callback for on bad packet?
  bool res = current_throttle.deserialize(data, length);

  Serial.printf("data cb %d\n", micros());
}

bool Comms::send(TelemetryPacket *packet) {
  // max packet is 228, anything getting serialized should fit below that
  static uint8_t data_buffer[228];
  size_t n = packet->serialize(data_buffer);
  return peer_link->sendData(data_buffer, n);
}

bool Comms::begin() {
  peer_link = new NowPair(NowPair::Role::Slave, 6, NETWORK_ID, HEARTBEAT_MS,
                          TIMEOUT_MS);

  peer_link->onData(dataCb, this);

  if (!peer_link->begin()) {
    end();
    return false;
  }
  return true;
}

void Comms::update() { peer_link->update(); }

bool Comms::connected() { return peer_link->connected(); }
