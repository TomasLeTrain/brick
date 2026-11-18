#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
constexpr int WIFI_IF_STA = 0;
struct esp_now_recv_info_t;
class ESP_NOW_Peer {
public:
  ESP_NOW_Peer(const uint8_t * = nullptr, uint8_t = 0, int = 0, const void * = nullptr) {}
  virtual ~ESP_NOW_Peer() = default;
  virtual void onSent(bool) {}
  const uint8_t *addr() const { return mac; }
  bool addr(const uint8_t *value) { std::memcpy(mac, value, 6); return true; }
protected:
  bool add() { return true; }
  size_t send(const uint8_t *, size_t size) { return size; }
private:
  uint8_t mac[6]{};
};
