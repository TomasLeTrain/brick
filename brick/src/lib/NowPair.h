#pragma once

#include <Arduino.h>
#include <ESP32_NOW.h>
#include <atomic>
#include <climits>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// One global/static instance per ESP32. All public methods and application
// callbacks run on the Arduino loop task. This class owns the ESP-NOW RX
// callback.
class NowPair : public ESP_NOW_Peer {
public:
  enum class Role : uint8_t { Master, Slave };
  static constexpr size_t MAX_PAYLOAD = 228;
  static constexpr int RSSI_UNAVAILABLE = INT_MIN;
  // using ConnectionCallback = void (*)(bool connected);
  using ConnectionCallback = void (*)(bool connected, void *arg);
  // RSSI is measured on the receiving board, in dBm, for this packet.
  using DataCallback = void (*)(const uint8_t *data, size_t length, int rssiDbm,
                                int rssiTimestamp, void *arg);

  explicit NowPair(Role role, uint8_t channel = 6,
                   uint32_t networkId = 0x12345678, uint32_t heartbeatMs = 500,
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

  // true means queued locally, NOT delivered/acknowledged by the application.
  bool sendData(const void *data, size_t length);
  uint32_t droppedRxPackets() const { return _droppedRx.load(); }
  bool lastMacSendSucceeded() const { return _lastTxSuccess.load(); }

private:
  enum class Kind : uint8_t { Discover = 1, Probe, Reply, Data };
  // ESP32-to-ESP32 little-endian wire format. Transmit header + used payload
  // only.
  struct __attribute__((packed)) Frame {
    uint32_t magic;
    uint32_t network;
    uint32_t sender;
    uint32_t target;
    uint32_t sequence;
    Kind kind;
    uint8_t length;
    uint8_t payload[MAX_PAYLOAD];
  };
  static constexpr size_t HEADER_SIZE = 22;
  static_assert(sizeof(Frame) == 250, "Unexpected wire layout");
  struct RxEvent {
    uint8_t mac[6];
    bool broadcast;
    uint32_t receivedAt;
    int rssiDbm;
    int rssiTimestamp;
    Frame frame;
  };
  class Broadcaster : public ESP_NOW_Peer {
  public:
    Broadcaster(NowPair &owner, uint8_t channel);
    bool start() { return add(); }
    size_t transmit(const uint8_t *data, int length) {
      return send(data, length);
    }
    void onSent(bool success) override { _owner.onSent(success); }

  private:
    NowPair &_owner;
  };

  void onSent(bool success) override;
  static void receivePacket(const esp_now_recv_info_t *info,
                            const uint8_t *data, int length);
  void enqueueRx(const uint8_t *mac, const uint8_t *data, size_t length,
                 bool broadcast, int rssiDbm, int rssiTimestamp);
  bool bind(const uint8_t *mac);
  void handle(const RxEvent &event);
  void setConnected(bool value);
  void adoptSession(uint32_t session);
  Frame makeFrame(Kind kind, uint32_t sequence = 0) const;
  bool queueControl(Kind kind, uint32_t sequence = 0);
  void serviceTx();

  static std::atomic<NowPair *> _receiver;
  static const uint8_t EMPTY_MAC[6];
  static const uint8_t BROADCAST_MAC[6];
  static constexpr uint32_t MAGIC = 0x4E505231; // "NPR1"
  Role _role;
  uint8_t _channel;
  uint32_t _network, _heartbeatMs, _timeoutMs;
  Broadcaster _broadcaster;
  QueueHandle_t _rx = nullptr, _controlTx = nullptr, _dataTx = nullptr;
  bool _started = false, _bound = false, _connected = false;
  uint32_t _session = 0, _remoteSession = 0;
  uint32_t _sequence = 0, _pendingSequence = 0;
  bool _probePending = false;
  uint32_t _lastReply = 0, _lastProbe = 0, _lastDiscovery = 0;
  std::atomic<bool> _txBusy{false}, _lastTxSuccess{false};
  std::atomic<uint32_t> _droppedRx{0};
  ConnectionCallback _connectionCallback = nullptr;
  void *_connectionArg = nullptr;
  DataCallback _dataCallback = nullptr;
  void *_dataArg = nullptr;
};
