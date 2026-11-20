#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

extern uint32_t host_ms;
extern bool host_motion;
extern unsigned host_flash_operations;
inline uint32_t millis() { return host_ms; }
inline uint32_t micros() { return host_ms * 1000; }
inline void pinMode(int, int) {}
inline int digitalRead(int) { return 1; }
constexpr int INPUT_PULLUP = 1, LOW = 0;
inline void flash_operation() {
  if (host_motion)
    throw std::runtime_error("filesystem touched while motor outputs active");
  ++host_flash_operations;
}

struct HostSerial {
  std::string *sink = nullptr;
  int room = 64;
  std::string input;
  size_t input_position = 0;
  int available() const { return int(input.size() - input_position); }
  int read() { return available() ? int(static_cast<unsigned char>(input[input_position++])) : -1; }
  int availableForWrite() const { return room; }
  size_t write(const uint8_t *data, size_t length) {
    if (length > size_t(room))
      throw std::runtime_error("serial write exceeds available capacity");
    sink->append(reinterpret_cast<const char *>(data), length);
    return length;
  }
  template <typename... Args> void printf(Args...) {
    throw std::runtime_error("blocking serial printf");
  }
  void print(const char *) { throw std::runtime_error("blocking serial print"); }
  void println(const char * = "") { throw std::runtime_error("blocking serial println"); }
};
extern HostSerial Serial;
