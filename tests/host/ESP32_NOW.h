#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
constexpr int WIFI_IF_STA = 0;
struct HostRxControl { int rssi; uint32_t timestamp; };
struct esp_now_recv_info_t {
  const uint8_t *src_addr, *des_addr;
  const HostRxControl *rx_ctrl;
};
using HostReceive = void (*)(const esp_now_recv_info_t *, const uint8_t *, int);
inline HostReceive host_receive = nullptr;
inline int esp_now_register_recv_cb(HostReceive callback) {
  host_receive = callback;
  return 0;
}
class ESP_NOW_Peer;
inline size_t (*host_transmit)(ESP_NOW_Peer &, const uint8_t *, size_t) = nullptr;
struct HostEspNow {
  bool begin() { return true; }
  void end() {}
};
inline HostEspNow ESP_NOW;
class ESP_NOW_Peer {
public:
  ESP_NOW_Peer(const uint8_t * = nullptr, uint8_t = 0, int = 0, const void * = nullptr) {}
  virtual ~ESP_NOW_Peer() = default;
  virtual void onSent(bool) {}
  const uint8_t *addr() const { return mac; }
  bool addr(const uint8_t *value) { std::memcpy(mac, value, 6); return true; }
protected:
  bool add() { return true; }
  size_t send(const uint8_t *data, size_t size) {
    return host_transmit ? host_transmit(*this, data, size) : size;
  }
private:
  uint8_t mac[6]{};
};
