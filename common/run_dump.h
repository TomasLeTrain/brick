#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace RunDump {
constexpr uint32_t magic = 0x31524442; // BDR1, little-endian.
constexpr uint32_t version = 3;
constexpr uint8_t protocol_version = 1;
constexpr size_t capacity = 2048;

// Fixed-width little-endian IEEE-754 format shared by file, radio and host tools.
struct Record {
  uint32_t timestamp_us;
  uint32_t received_us; // Local task-delivery time, including queue delay.
  float ax_g, ay_g, az_g;
  float target_rpm, rssi_rpm, control_rpm, rssi_hz, throttle;
  uint32_t estimator_us, rx_dropped, rssi_dropped;
  int16_t rssi_dbm;
  uint8_t kind; // 1 = RSSI, 2 = accelerometer/control.
  // v3 bit 7 identifies diagnostics, bit 0 Ready, bits 1..3 connection state,
  // bit 4 controller alive, bit 5 spin switch, bit 6 capture switch.
  uint8_t connected;
  float fast_rpm, long_rpm, correlation;
  uint32_t analysis_timestamp_us;
  uint16_t comparison_samples, min_lag, max_lag;
  uint8_t tracking_mode, hint_used; // v3 tracking_mode bits 2..7 store RPM status.
  uint32_t motor_gap_us;
};
static_assert(sizeof(Record) == 84, "Dump record layout changed");

struct Header {
  uint32_t magic_value, format_version, generation, count, record_size, crc;
  uint32_t overwritten, paused_samples;
};
static_assert(sizeof(Header) == 32, "Dump header layout changed");

inline bool supported_format(const Header &header) {
  return header.magic_value == magic && header.count <= capacity &&
      ((header.format_version == 1 && header.record_size == 56) ||
       (header.format_version == 2 && header.record_size == 80) ||
       (header.format_version == version && header.record_size == sizeof(Record)));
}

inline bool decode_record(const uint8_t *data, const Header &header, Record &record) {
  if (!supported_format(header))
    return false;
  record = {};
  std::memcpy(&record, data, header.record_size);
  if (header.format_version == 1)
    record.target_rpm = 0; // The old slot held accelerometer RPM.
  return true;
}

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
    "kind,timestamp_us,received_us,rssi_dbm,ax_g,ay_g,az_g,target_rpm,"
    "rssi_rpm,control_rpm,rssi_hz,throttle,estimator_us,rx_dropped,"
    "rssi_dropped,connected,fast_rpm,long_rpm,correlation,analysis_timestamp_us,"
    "comparison_samples,min_lag,max_lag,tracking_mode,hint_used,"
    "connection_state,controller_connected,spin_enabled,capture_switch,rpm_status,motor_gap_us\n";

inline int csv_record(char *buffer, size_t size, const Record &r) {
  return std::snprintf(buffer, size,
      "%u,%lu,%lu,%d,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%lu,%lu,%lu,%u,%.5g,%.5g,%.5g,%lu,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%lu\n",
      unsigned(r.kind), static_cast<unsigned long>(r.timestamp_us),
      static_cast<unsigned long>(r.received_us), int(r.rssi_dbm),
      r.ax_g, r.ay_g, r.az_g, r.target_rpm, r.rssi_rpm, r.control_rpm,
      r.rssi_hz, r.throttle, static_cast<unsigned long>(r.estimator_us),
      static_cast<unsigned long>(r.rx_dropped),
      static_cast<unsigned long>(r.rssi_dropped), unsigned(r.connected & 1u),
      r.fast_rpm, r.long_rpm, r.correlation,
      static_cast<unsigned long>(r.analysis_timestamp_us),
      unsigned(r.comparison_samples), unsigned(r.min_lag), unsigned(r.max_lag),
      unsigned(r.tracking_mode & 3u), unsigned(r.hint_used),
      r.connected & 0x80 ? (r.connected >> 1) & 7 : -1,
      r.connected & 0x80 ? bool(r.connected & 0x10) : -1,
      r.connected & 0x80 ? bool(r.connected & 0x20) : -1,
      r.connected & 0x80 ? bool(r.connected & 0x40) : -1,
      r.connected & 0x80 ? r.tracking_mode >> 2 : -1,
      static_cast<unsigned long>(r.motor_gap_us));
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
         data[2] == protocol_version;
}
inline void prefix(uint8_t *data, Message message, uint32_t session,
                   uint32_t index) {
  data[0] = 'B'; data[1] = 'D'; data[2] = protocol_version;
  data[3] = static_cast<uint8_t>(message);
  put_u32(data + 4, session);
  put_u32(data + 8, index);
}
} // namespace RunDump
