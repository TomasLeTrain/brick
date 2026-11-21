#ifndef BRICK_NOW_PAIR_H
#define BRICK_NOW_PAIR_H

#pragma once

#include <Arduino.h>
#include <ESP32_NOW.h>
#include <atomic>
#include <climits>
#include <cstddef>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// One global/static instance per ESP32. All public methods and application
// callbacks run on the task that services update(). This class owns the ESP-NOW
// RX callback.
class NowPair : public ESP_NOW_Peer {
public:
  enum class Role : uint8_t { Master, Slave };
  static constexpr size_t MAX_PAYLOAD = 120;
  static constexpr int RSSI_UNAVAILABLE = INT_MIN;
  using RealtimeCallback = void (*)(const uint8_t *, size_t,
                                    uint32_t receivedMs, void *);
  using ConnectionCallback = void (*)(bool connected, void *arg);
  // RSSI is measured on the receiving board, in dBm, for this packet.
  using DataCallback = void (*)(const uint8_t *data, size_t length, int rssiDbm,
                                uint32_t timestampUs, void *arg);
  using RssiCallback = void (*)(int rssiDbm, uint32_t timestampUs, void *arg);
  using TimedRssiCallback = void (*)(int rssiDbm, uint32_t radioUs,
                                     uint32_t callbackUs, void *arg);

  explicit NowPair(Role role, uint8_t channel = 6, uint32_t heartbeatMs = 500,
                   uint32_t timeoutMs = 3000);
  NowPair(const NowPair &) = delete;
  NowPair &operator=(const NowPair &) = delete;
  NowPair(NowPair &&) = delete;
  NowPair &operator=(NowPair &&) = delete;

  // Configures standalone station Wi-Fi and starts ESP-NOW. Call once in setup.
  bool begin();
  // Call frequently; processes reception, timers, callbacks, and queued sends.
  void update();
  bool connected() const { return _connected; }
  // nullptr until a remote has been discovered. MAC stays fixed until reboot.
  const uint8_t *remoteAddress() const { return _bound ? addr() : nullptr; }

  void onConnectionChange(ConnectionCallback cb, void *arg) {
    _connectionCallback = cb;
    _connectionArg = arg;
  }
  void onData(DataCallback cb, void *arg) {
    _dataCallback = cb;
    _dataArg = arg;
  }
  // Samples from the bound peer are delivered by update(), even when
  // disconnected. Pass nullptr as the callback to stop collecting samples.
  void onRssi(RssiCallback cb, void *arg) {
    _rssiCallback = cb;
    _rssiArg = arg;
    _collectRssi.store(cb != nullptr || _timedRssiCallback != nullptr);
  }
  // callbackUs is captured on the Wi-Fi task before queueing, in micros()'s
  // clock domain. Phase tracking must not mistake queue latency for rotation.
  void onRssiTimed(TimedRssiCallback cb, void *arg) {
    _timedRssiCallback = cb;
    _timedRssiArg = arg;
    _collectRssi.store(cb != nullptr || _rssiCallback != nullptr);
  }

  // true means queued locally, NOT delivered/acknowledged by the application.
  bool sendData(const void *data, size_t length);
  // Queue an empty measurement packet once the remote MAC has been learned.
  bool sendRssiSample();
  // Latest-value control mailbox, ahead of bulk data, expired after 100 ms.
  bool sendLatestData(const void *data, size_t length);
  void onRealtime(RealtimeCallback cb, void *arg) {
    _realtimeCallback = cb;
    _realtimeArg = arg;
  }
  // Called by the Wi-Fi task only to wake the application, never to send again.
  void wakeOnSend(TaskHandle_t task) { _txWakeTask = task; }
  void expectPeer(const uint8_t mac[6]) {
    if (!_started)
      memcpy(_expectedPeer, mac, 6);
  }
  struct TxStats {
    uint32_t attempts, completed, failures, max_completion_us;
  };
  TxStats txStats() const {
    return {_txAttempts.load(), _txCompleted.load(), _txFailures.load(),
            _txMaxUs.load()};
  }
  // Fill idle TX slots with RSSI packets while connected. Control and
  // application data still take priority, with only one frame in flight.
  void setRssiStreaming(bool enabled) { _streamRssi = enabled; }
  uint32_t droppedRxPackets() const { return _droppedRx.load(); }
  uint32_t droppedRssiSamples() const { return _droppedRssi.load(); }
  uint32_t droppedControlReplies() const { return _droppedReplies; }
  bool lastMacSendSucceeded() const { return _lastTxSuccess.load(); }

private:
  enum class Kind : uint8_t {
    Discover = 1,
    Probe,
    Reply,
    Data,
    Rssi,
    Realtime
  };

  struct __attribute__((packed)) Frame {
    Kind kind;
    uint8_t length;
    uint8_t payload[MAX_PAYLOAD];
  };

  static constexpr size_t HEADER_SIZE = 2;
  static_assert(offsetof(Frame, payload) == HEADER_SIZE,
                "Unexpected header layout");
  static_assert(sizeof(Frame) == HEADER_SIZE + MAX_PAYLOAD,
                "Unexpected frame layout");

  struct RssiSample {
    int rssiDbm;
    uint32_t timestampUs;
    uint32_t callbackUs;
  };

  struct RxEvent {
    uint8_t mac[6];
    uint32_t receivedAt;
    RssiSample sample;
    Frame frame;
  };

  // used to broadcast discovery packets to everywhere in channel
  class Broadcaster : public ESP_NOW_Peer {
  public:
    Broadcaster(NowPair &owner, uint8_t channel);
    bool start() { return add(); }
    size_t transmit(const uint8_t *data, int length) {
      return send(data, length);
    }
    // forwards call to owner
    void onSent(bool success) override { _owner.onSent(success); }

  private:
    NowPair &_owner;
  };

  void onSent(bool success) override;
  static void receivePacket(const esp_now_recv_info_t *info,
                            const uint8_t *data, int length);
  void enqueueRx(const uint8_t *mac, const uint8_t *data, size_t length,
                 bool broadcast, const RssiSample &sample);
  bool bind(const uint8_t *mac);
  void handle(const RxEvent &event);
  void setConnected(bool value);
  void resetLink(uint32_t now);
  Frame makeFrame(Kind kind) const;

  // used for zero-payload control packets
  bool queueControl(Kind kind);

  // used for probe/reply packets
  bool queueControlSequence(Kind kind, uint32_t sequence);

  void serviceTx();

  static std::atomic<NowPair *> _receiver;
  static const uint8_t EMPTY_MAC[6];
  static const uint8_t BROADCAST_MAC[6];
  Role _role;
  uint8_t _channel;
  uint32_t _heartbeatMs, _timeoutMs;
  Broadcaster _broadcaster;
  QueueHandle_t _rx = nullptr, _rssiRx = nullptr, _realtimeRx = nullptr;
  QueueHandle_t _controlTx = nullptr, _dataTx = nullptr;
  bool _started = false, _connected = false;
  std::atomic<bool> _bound{false};
  uint32_t _sequence = 0, _pendingSequence = 0;
  bool _probePending = false;
  uint32_t _lastReply = 0, _lastProbe = 0, _lastDiscovery = 0;
  std::atomic<bool> _txBusy{false}, _lastTxSuccess{false};
  Frame _latest{};
  bool _latestPending = false;
  uint32_t _latestAt = 0;
  uint8_t _expectedPeer[6]{};
  TaskHandle_t _txWakeTask = nullptr;
  std::atomic<uint32_t> _txAttempts{0}, _txCompleted{0}, _txFailures{0};
  std::atomic<uint32_t> _txStartedUs{0}, _txMaxUs{0};
  RealtimeCallback _realtimeCallback = nullptr;
  void *_realtimeArg = nullptr;
  bool _streamRssi = false;
  std::atomic<bool> _collectRssi{false};
  std::atomic<uint32_t> _droppedRx{0}, _droppedRssi{0};
  uint32_t _droppedReplies = 0;
  ConnectionCallback _connectionCallback = nullptr;
  void *_connectionArg = nullptr;
  DataCallback _dataCallback = nullptr;
  void *_dataArg = nullptr;
  RssiCallback _rssiCallback = nullptr;
  void *_rssiArg = nullptr;
  TimedRssiCallback _timedRssiCallback = nullptr;
  void *_timedRssiArg = nullptr;
};

#endif // BRICK_NOW_PAIR_H
