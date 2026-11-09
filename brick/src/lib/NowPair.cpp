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
  _rssiRx = xQueueCreate(64, sizeof(RssiSample));
  _controlTx = xQueueCreate(8, sizeof(Frame));
  _dataTx = xQueueCreate(8, sizeof(Frame));
  if (!_rx || !_rssiRx || !_controlTx || !_dataTx) {
    if (_rx)
      vQueueDelete(_rx);
    if (_rssiRx)
      vQueueDelete(_rssiRx);
    if (_controlTx)
      vQueueDelete(_controlTx);
    if (_dataTx)
      vQueueDelete(_dataTx);
    _rx = _rssiRx = _controlTx = _dataTx = nullptr;
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
    vQueueDelete(_rssiRx);
    vQueueDelete(_controlTx);
    vQueueDelete(_dataTx);
    _rx = _rssiRx = _controlTx = _dataTx = nullptr;
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
  const RssiSample sample =
      info->rx_ctrl ? RssiSample{info->rx_ctrl->rssi, info->rx_ctrl->timestamp}
                    : RssiSample{RSSI_UNAVAILABLE, 0};
  self->enqueueRx(info->src_addr, data, static_cast<size_t>(length),
                  memcmp(info->des_addr, BROADCAST_MAC, 6) == 0, sample);
}

// Wi-Fi task: only copy bounded data into a FreeRTOS queue. No user callbacks.
void NowPair::enqueueRx(const uint8_t *mac, const uint8_t *data, size_t length,
                        bool broadcast, const RssiSample &sample) {
  if (!_rx || !data || length < HEADER_SIZE || length > sizeof(Frame))
    return;

  const auto kind = static_cast<Kind>(data[0]);
  const size_t payloadLength = data[1];
  if (length != HEADER_SIZE + payloadLength ||
      broadcast != (kind == Kind::Discover))
    return;

  switch (kind) {
  case Kind::Discover:
  case Kind::Rssi:
    if (payloadLength != 0)
      return;
    break;
  case Kind::Probe:
  case Kind::Reply:
    if (payloadLength != sizeof(uint32_t))
      return;
    break;
  case Kind::Data:
    break;
  default:
    return;
  }

  // Capture usable metadata independently of pairing and heartbeat state.
  if (_collectRssi.load() && sample.rssiDbm != RSSI_UNAVAILABLE &&
      xQueueSend(_rssiRx, &sample, 0) != pdTRUE)
    _droppedRssi.fetch_add(1);
  if (kind == Kind::Rssi)
    return;

  RxEvent event{};
  memcpy(&event.frame, data, length);
  memcpy(event.mac, mac, 6);
  event.receivedAt = millis();
  event.sample = sample;

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
  _lastProbe = millis() - _heartbeatMs;
  return true;
}

void NowPair::setConnected(bool value) {
  if (_connected == value)
    return;
  _connected = value;
  if (_connectionCallback)
    _connectionCallback(value, _connectionArg);
}

void NowPair::resetLink(uint32_t now) {
  _probePending = false;
  xQueueReset(_controlTx);
  xQueueReset(_dataTx);
  _lastProbe = now - _heartbeatMs;
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

bool NowPair::queueControlSequence(Kind kind, uint32_t sequence) {
  Frame f = makeFrame(kind);
  f.length = sizeof(sequence);
  memcpy(f.payload, &sequence, sizeof(sequence));
  return xQueueSend(_controlTx, &f, 0) == pdTRUE;
}

void NowPair::handle(const RxEvent &event) {
  // Do not establish/refresh links using an event left in the queue too long.
  if (uint32_t(millis() - event.receivedAt) >= _timeoutMs)
    return;
  const Frame &f = event.frame;
  if (f.kind == Kind::Discover) {
    if (_role == Role::Slave)
      bind(event.mac);
    return;
  }

  if (_bound) {
    if (memcmp(addr(), event.mac, 6) != 0)
      return;
  } else {
    // Only the master may learn an unknown slave from its first unicast probe.
    if (_role != Role::Master || f.kind != Kind::Probe || !bind(event.mac))
      return;
  }
  switch (f.kind) {
  case Kind::Probe:
  case Kind::Reply: {
    uint32_t sequence;
    memcpy(&sequence, f.payload, sizeof(sequence));
    if (f.kind == Kind::Probe) {
      if (!queueControlSequence(Kind::Reply, sequence))
        ++_droppedReplies;
    } else if (_probePending && sequence == _pendingSequence) {
      _probePending = false;
      _lastReply = event.receivedAt;
      setConnected(true);
    }
    break;
  }
  case Kind::Data:
    if (_connected && _dataCallback)
      _dataCallback(f.payload, f.length, event.sample.rssiDbm,
                    event.sample.timestampUs, _dataArg);
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

bool NowPair::sendRssiSample() {
  if (!_started || !_bound)
    return false;
  const Frame f = makeFrame(Kind::Rssi);
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

  // Discard frames whose destination or connection state no longer permits
  // them.
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

  // Process the backlog present at entry. New arrivals cannot extend this pass.
  const auto pendingRx = uxQueueMessagesWaiting(_rx);
  RxEvent event;
  for (UBaseType_t i = 0;
       i < pendingRx && xQueueReceive(_rx, &event, 0) == pdTRUE; ++i)
    handle(event);

  const uint32_t now = millis();

  if (_connected && uint32_t(now - _lastReply) >= _timeoutMs)
    resetLink(now);

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

  // Give protocol traffic and transmission a turn before dispatching samples.
  const auto pendingSamples = uxQueueMessagesWaiting(_rssiRx);
  RssiSample sample;
  for (UBaseType_t i = 0;
       i < pendingSamples && xQueueReceive(_rssiRx, &sample, 0) == pdTRUE;
       ++i) {
    if (_rssiCallback)
      _rssiCallback(sample.rssiDbm, sample.timestampUs, _rssiArg);
  }
}
