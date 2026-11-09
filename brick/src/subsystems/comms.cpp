#include "comms.h"

Comms::Comms() {}
Comms::~Comms() { end(); }

void Comms::end() {
  if (peer_link) {
    delete peer_link;
  }
}

void Comms::dataCb(const uint8_t *data, size_t length, int rssiDbm,
                   int rssiTimestamp, void *arg) {
  if (data == nullptr || length < 0)
    return;

  auto *self = static_cast<Comms *>(arg);

  self->on_data(data, length, rssiDbm, rssiTimestamp);
}

void Comms::on_data(const uint8_t *data, size_t length, int rssiDbm,
                    int rssiTimestamp) {
  // TODO: should definetely do something with res
  // maybe increase packet loss, or have callback for on bad packet?
  bool res = current_throttle.deserialize(data, length);
  new_rssi_entry(rssiDbm, rssiTimestamp);
}

bool Comms::send(TelemetryPacket *packet) {
  // max packet is 228, anything getting serialized should fit below that
  static uint8_t data_buffer[228];
  size_t n = packet->serialize(data_buffer);

  return peer_link->sendData(data_buffer, n);
}

bool Comms::begin() {
  peer_link = new NowPair(NowPair::Role::Slave, 6, HEARTBEAT_MS, TIMEOUT_MS);

  peer_link->onData(dataCb, this);

  if (!peer_link->begin()) {
    end();
    return false;
  }
  return true;
}

void Comms::update() { peer_link->update(); }

bool Comms::connected() { return peer_link->connected(); }

void Comms::new_rssi_entry(int rssiDbm, int rssiTimestamp) {
  if (rssiDbm == NowPair::RSSI_UNAVAILABLE || dumping_rssi)
    return;

  rssi[rssi_idx] = rssiDbm;
  rssi_timestamps[rssi_idx] = rssiTimestamp;

  rssi_idx = (rssi_idx + 1) % RSSI_QUEUE_SIZE;
  if (rssi_capacity < RSSI_QUEUE_SIZE)
    rssi_capacity++;
}

void Comms::dump_rssi() {
  dumping_rssi = true;

  const uint32_t NVM_Offset = 0x9000;
  uint32_t address = 0;

  int total_len = sizeof(rssi) + sizeof(rssi_timestamps);

  for (int sector = (NVM_Offset + address) / 4096;
       sector <= (NVM_Offset + address + total_len) / 4096; sector++) {
    ESP.flashEraseSector(sector);
  }

  ESP.flashWrite(NVM_Offset + address, (uint32_t *)rssi, sizeof(rssi));

  address += sizeof(rssi);

  ESP.flashWrite(NVM_Offset + address, (uint32_t *)rssi_timestamps,
                 sizeof(rssi_timestamps));

  dumping_rssi = false;
}

void Comms::print_dump() {
  // TODO:
}
