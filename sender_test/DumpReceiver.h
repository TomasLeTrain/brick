#pragma once

#include "../common/run_dump.h"
#include "NowPair.h"
#include <algorithm>

// CSV output is drained without blocking the radio/IBus loop. ACKs follow the
// completed row, so retransmitted records cannot duplicate partially printed CSV.
class DumpReceiver {
public:
  bool receive(const uint8_t *data, size_t length) {
    if (!RunDump::is_message(data, length))
      return false;
    const auto type = static_cast<RunDump::Message>(data[3]);
    const uint32_t incoming_session = RunDump::get_u32(data + 4);
    const uint32_t index = RunDump::get_u32(data + 8);
    if (type == RunDump::Message::Status) {
      if (!printing() && !active)
        queue("# dump %.*s\n", int(length - RunDump::prefix_size),
              reinterpret_cast<const char *>(data + RunDump::prefix_size));
      return true;
    }
    last_received_ms = millis();
    if (type == RunDump::Message::Begin &&
        length == RunDump::prefix_size + sizeof(RunDump::Header)) {
      RunDump::Header header;
      std::memcpy(&header, data + RunDump::prefix_size, sizeof(header));
      if (!RunDump::supported_format(header))
        return true;
      if (!have_session || session != incoming_session) {
        if (printing())
          return true;
        session = incoming_session;
        have_session = active = true;
        next = 0;
        checksum = UINT32_MAX;
        expected = header;
        queue("# dump_begin generation=%lu count=%lu overwritten=%lu paused=%lu\n%s",
              static_cast<unsigned long>(header.generation),
              static_cast<unsigned long>(header.count),
              static_cast<unsigned long>(header.overwritten),
              static_cast<unsigned long>(header.paused_samples), RunDump::csv_header);
      }
      acknowledge(RunDump::begin_index);
    } else if (have_session && session == incoming_session) {
      if (type == RunDump::Message::Record &&
          length == RunDump::prefix_size + expected.record_size) {
        if (active && index == next && next < expected.count && !printing()) {
          RunDump::Record record;
          RunDump::decode_record(data + RunDump::prefix_size, expected, record);
          checksum = RunDump::crc32(checksum, data + RunDump::prefix_size,
                                   expected.record_size);
          output_size = RunDump::csv_record(output, sizeof(output), record);
          output_position = 0;
          ++next;
        }
        if (next > 0 && index == next - 1)
          acknowledge(index);
      } else if (type == RunDump::Message::End &&
                 length == RunDump::prefix_size && next == expected.count) {
        if ((checksum ^ UINT32_MAX) == expected.crc) {
          if (active && !printing()) {
            queue("# dump_end\n");
            active = false;
          }
          acknowledge(RunDump::end_index);
        } else if (!printing()) {
          queue("# dump checksum mismatch; request e to restart\n");
          active = false;
        }
      }
    }
    return true;
  }

  void update(NowPair &link) {
    const size_t bytes = std::min(output_size - output_position,
        size_t(std::max(0, Serial.availableForWrite())));
    if (bytes)
      output_position += Serial.write(
          reinterpret_cast<const uint8_t *>(output + output_position), bytes);
    if (active && (!link.connected() || millis() - last_received_ms > 3000)) {
      active = false;
      pending_ack = false;
      // A partial row is allowed to drain. Missing dump_end marks interruption.
    }
    if (pending_ack && !printing() && link.sendData(ack, sizeof(ack)))
      pending_ack = false;
  }
  bool receiving() const { return active; }
  bool printing() const { return output_position < output_size; }

private:
  template <typename... Args> void queue(const char *format, Args... args) {
    output_size = std::snprintf(output, sizeof(output), format, args...);
    output_position = 0;
  }
  void acknowledge(uint32_t index) {
    RunDump::prefix(ack, RunDump::Message::Ack, session, index);
    pending_ack = true;
  }
  bool have_session = false, active = false, pending_ack = false;
  uint32_t session = 0, next = 0, checksum = UINT32_MAX, last_received_ms = 0;
  RunDump::Header expected{};
  uint8_t ack[RunDump::prefix_size];
  char output[768];
  size_t output_size = 0, output_position = 0;
};
