#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace RunDump {
constexpr uint32_t magic = 0x31524442; // BDR1, little-endian.
constexpr uint32_t version = 1;
constexpr size_t capacity = 2048;

// Fixed-width little-endian IEEE-754 format shared by file, radio and host tools.
struct Record {
  uint32_t timestamp_us;
  uint32_t received_us; // Local task-delivery time, including queue delay.
  float ax_g, ay_g, az_g;
  float accel_rpm, rssi_rpm, control_rpm, rssi_hz, throttle;
  uint32_t estimator_us, rx_dropped, rssi_dropped;
  int16_t rssi_dbm;
  uint8_t kind; // 1 = RSSI, 2 = accelerometer/control.
  uint8_t connected;
};
static_assert(sizeof(Record) == 56, "Dump record layout changed");

struct Header {
  uint32_t magic_value, format_version, generation, count, record_size, crc;
  uint32_t overwritten, paused_samples;
};
static_assert(sizeof(Header) == 32, "Dump header layout changed");

inline uint32_t crc32(uint32_t crc, const void *data, size_t length) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return crc;
}

constexpr const char *csv_header =
    "kind,timestamp_us,received_us,rssi_dbm,ax_g,ay_g,az_g,accel_rpm,"
    "rssi_rpm,control_rpm,rssi_hz,throttle,estimator_us,rx_dropped,"
    "rssi_dropped,connected\n";

inline int csv_record(char *buffer, size_t size, const Record &r) {
  return std::snprintf(buffer, size,
      "%u,%lu,%lu,%d,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%lu,%lu,%lu,%u\n",
      unsigned(r.kind), static_cast<unsigned long>(r.timestamp_us),
      static_cast<unsigned long>(r.received_us), int(r.rssi_dbm),
      r.ax_g, r.ay_g, r.az_g, r.accel_rpm, r.rssi_rpm, r.control_rpm,
      r.rssi_hz, r.throttle, static_cast<unsigned long>(r.estimator_us),
      static_cast<unsigned long>(r.rx_dropped),
      static_cast<unsigned long>(r.rssi_dropped), unsigned(r.connected));
}

enum class Message : uint8_t { Command = 1, Begin, Record, End, Ack, Status };
constexpr size_t prefix_size = 12;
constexpr uint32_t begin_index = UINT32_MAX;
constexpr uint32_t end_index = UINT32_MAX - 1;

inline void put_u32(uint8_t *out, uint32_t value) {
  for (unsigned i = 0; i < 4; ++i)
    out[i] = value >> (8 * i);
}
inline uint32_t get_u32(const uint8_t *in) {
  uint32_t value = 0;
  for (unsigned i = 0; i < 4; ++i)
    value |= uint32_t(in[i]) << (8 * i);
  return value;
}
inline bool is_message(const uint8_t *data, size_t length) {
  return length >= prefix_size && data[0] == 'B' && data[1] == 'D' &&
         data[2] == version;
}
inline void prefix(uint8_t *data, Message message, uint32_t session,
                   uint32_t index) {
  data[0] = 'B'; data[1] = 'D'; data[2] = version;
  data[3] = static_cast<uint8_t>(message);
  put_u32(data + 4, session);
  put_u32(data + 8, index);
}
} // namespace RunDump
