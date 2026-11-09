#include "NowPair.h"
#include <WiFi.h>
#include <esp_system.h>
#include <esp_wifi.h>

std::atomic<NowPair *> NowPair::_receiver{nullptr};
const uint8_t NowPair::EMPTY_MAC[6] = {};
const uint8_t NowPair::BROADCAST_MAC[6] = {255, 255, 255, 255, 255, 255};

NowPair::Broadcaster::Broadcaster(NowPair &owner, uint8_t channel)
    : ESP_NOW_Peer(BROADCAST_MAC, channel, WIFI_IF_STA, nullptr),
      _owner(owner) {}

NowPair::NowPair(Role role, uint8_t channel, uint32_t heartbeatMs,
                 uint32_t timeoutMs)
    : ESP_NOW_Peer(EMPTY_MAC, channel, WIFI_IF_STA, nullptr), _role(role),
      _channel(channel), _heartbeatMs(heartbeatMs), _timeoutMs(timeoutMs),
      _broadcaster(*this, channel) {}

bool NowPair::begin() {
  if (_started)
    return true;
  if (_receiver.load() || _channel < 1 || _channel > 11 || _heartbeatMs < 20 ||
      _timeoutMs <= _heartbeatMs || _timeoutMs >= 0x80000000UL)
    return false;

  _rx = xQueueCreate(12, sizeof(RxEvent));
  _controlTx = xQueueCreate(8, sizeof(Frame));
  _dataTx = xQueueCreate(8, sizeof(Frame));
  if (!_rx || !_controlTx || !_dataTx) {
    if (_rx)
      vQueueDelete(_rx);
    if (_controlTx)
      vQueueDelete(_controlTx);
    if (_dataTx)
      vQueueDelete(_dataTx);
    _rx = _controlTx = _dataTx = nullptr;
    return false;
  }

  _receiver.store(this);

  // Install the metadata-aware receiver after ESP_NOW.begin() sets up its
  // callbacks. ESP_NOW_Peer still handles peer registration and transmission.
  if (!WiFi.mode(WIFI_STA) ||
      (esp_wifi_set_channel(_channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) ||
      !WiFi.setSleep(false) || !ESP_NOW.begin() || !_broadcaster.start() ||
      esp_now_register_recv_cb(receivePacket) != ESP_OK) {
    // This class exclusively owns ESP-NOW on this board.
    _receiver.store(nullptr);
    ESP_NOW.end();
    vQueueDelete(_rx);
    vQueueDelete(_controlTx);
    vQueueDelete(_dataTx);
    _rx = _controlTx = _dataTx = nullptr;
    return false;
  }
  _sequence = esp_random();
  _lastProbe = millis() - _heartbeatMs;
  _lastDiscovery = millis() - _heartbeatMs;
  _started = true;
  return true;
}

void NowPair::receivePacket(const esp_now_recv_info_t *info,
                            const uint8_t *data, int length) {
  NowPair *self = _receiver.load();
  if (!self || !info || !info->src_addr || !info->des_addr || length < 0)
    return;
  const int rssiDbm = info->rx_ctrl ? info->rx_ctrl->rssi : RSSI_UNAVAILABLE;
  const int rssiTimestamp =
      info->rx_ctrl ? info->rx_ctrl->timestamp : RSSI_UNAVAILABLE;

  self->enqueueRx(info->src_addr, data, static_cast<size_t>(length),
                  memcmp(info->des_addr, BROADCAST_MAC, 6) == 0, rssiDbm,
                  rssiTimestamp);
}

// Wi-Fi task: only copy bounded data into a FreeRTOS queue. No user callbacks.
void NowPair::enqueueRx(const uint8_t *mac, const uint8_t *data, size_t length,
                        bool broadcast, int rssiDbm, int rssiTimestamp) {
  if (!_rx || !data || length < HEADER_SIZE || length > sizeof(Frame))
    return;

  RxEvent event{};
  memcpy(&event.frame, data, length);
  const Frame &f = event.frame;
  if (f.length > MAX_PAYLOAD || length != HEADER_SIZE + f.length)
    return;

  memcpy(event.mac, mac, 6);
  event.broadcast = broadcast;
  event.receivedAt = millis();
  event.rssiDbm = rssiDbm;
  event.rssiTimestamp = rssiTimestamp;

  if (xQueueSend(_rx, &event, 0) != pdTRUE)
    _droppedRx.fetch_add(1);
}

void NowPair::onSent(bool success) {
  _lastTxSuccess.store(success);
  _txBusy.store(false);
}

bool NowPair::bind(const uint8_t *mac) {
  if (_bound)
    return memcmp(addr(), mac, 6) == 0;
  // Assign only while unregistered; keep the object/address stable afterwards.
  if ((mac[0] & 1) || memcmp(mac, EMPTY_MAC, 6) == 0)
    return false;
  addr(mac);
  if (!add())
    return false;
  _bound = true;
  return true;
}

void NowPair::setConnected(bool value) {
  if (_connected == value)
    return;
  _connected = value;
  if (_connectionCallback)
    _connectionCallback(value, _connectionArg);
}

void NowPair::newSession() {
  _probePending = false;
  xQueueReset(_controlTx);
  xQueueReset(_dataTx);
  _lastProbe = millis() - _heartbeatMs;
  setConnected(false);
}

NowPair::Frame NowPair::makeFrame(Kind kind) const {
  Frame f{};
  f.kind = kind;
  return f;
}

bool NowPair::queueControl(Kind kind) {
  const Frame f = makeFrame(kind);
  return xQueueSend(_controlTx, &f, 0) == pdTRUE;
}

bool NowPair::queueControlSequence(Kind kind, uint32_t sequence = 0) {
  Frame f = makeFrame(kind);
  SequenceData *data = reinterpret_cast<SequenceData *>(&f.payload);
  data.sequence = sequence;

  return xQueueSend(_controlTx, &f, 0) == pdTRUE;
}

void NowPair::handle(const RxEvent &event) {
  // Do not establish/refresh links using an event left in the queue too long.
  if (uint32_t(millis() - event.receivedAt) >= _timeoutMs)
    return;
  const Frame &f = event.frame;
  if (f.kind == Kind::Discover) {
    // only acknolwedge discovery if slave
    if (_role != Role::Slave || !event.broadcast)
      return;
    if (!bind(event.mac))
      return;
    newSession();
    return;
  }

  if (event.broadcast)
    return;

  if (!_bound) {
    // Only the master may learn an unknown slave from its first unicast probe.
    if (_role != Role::Master || f.kind != Kind::Probe || !bind(event.mac))
      return;
  }
  if (memcmp(addr(), event.mac, 6) != 0)
    return;
  if (f.kind == Kind::Probe && _role == Role::Master)
    newSession();

  switch (f.kind) {
  case Kind::Probe:
    SequenceData *data = reinterpret_cast<const SequenceData *>(&f.payload);
    queueControlSequence(Kind::Reply, data.sequence);
    break;
  case Kind::Reply:
    SequenceData *data = reinterpret_cast<const SequenceData *>(&f.payload);
    if (_probePending && data.sequence == _pendingSequence) {
      _probePending = false;
      _lastReply = event.receivedAt;
      setConnected(true);
    }
    break;
  case Kind::Data:
    if (_connected && _dataCallback)
      _dataCallback(f.payload, f.length, event.rssiDbm, event.rssiTimestamp,
                    _dataArg);
    break;
  default:
    break;
  }
}

bool NowPair::sendData(const void *data, size_t length) {
  if (!_started || !_connected || length > MAX_PAYLOAD || (!data && length))
    return false;
  Frame f = makeFrame(Kind::Data);
  f.length = static_cast<uint8_t>(length);
  if (length)
    memcpy(f.payload, data, length);
  return xQueueSend(_dataTx, &f, 0) == pdTRUE;
}

void NowPair::serviceTx() {
  if (_txBusy.load())
    return;
  Frame f;
  // no packets to service
  if (xQueueReceive(_controlTx, &f, 0) != pdTRUE &&
      xQueueReceive(_dataTx, &f, 0) != pdTRUE)
    return;

  const bool broadcast = f.kind == Kind::Discover;

  // Discard work from an old peer boot/session or a now-disconnected app.
  if (!broadcast && !_bound)
    return;
  if (f.kind == Kind::Data && !_connected)
    return;
  if (broadcast && _connected)
    return;
  const size_t length = HEADER_SIZE + f.length;
  _txBusy.store(true); // callback may run before send() returns
  const size_t sent =
      broadcast
          ? _broadcaster.transmit(reinterpret_cast<const uint8_t *>(&f), length)
          : ESP_NOW_Peer::send(reinterpret_cast<const uint8_t *>(&f), length);
  if (sent != length) {
    _lastTxSuccess.store(false);
    _txBusy.store(
        false); // immediate failure: no completion callback is expected
  }
}

void NowPair::update() {
  if (!_started)
    return;

  // handle all received rx events
  RxEvent event;
  while (xQueueReceive(_rx, &event, 0) != pdTRUE)
    handle(event);

  const uint32_t now = millis();

  // timed out, start new session
  // TODO: is it necesary to reject old data?
  if (_connected && uint32_t(now - _lastReply) >= _timeoutMs) {
    newSession();
  }

  // send discover packet if master and has no connect
  if (_role == Role::Master && !_connected &&
      uint32_t(now - _lastDiscovery) >= _heartbeatMs) {
    if (queueControl(Kind::Discover))
      _lastDiscovery = now;
  }

  // send out heartbeat
  if (_bound && uint32_t(now - _lastProbe) >= _heartbeatMs) {
    // sequence stays the same until acknowledged
    if (!_probePending)
      _pendingSequence = ++_sequence;
    if (queueControlSequence(Kind::Probe, _pendingSequence)) {
      _probePending = true;
      _lastProbe = now;
    }
  }

  serviceTx();
}
