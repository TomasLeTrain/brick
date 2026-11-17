#pragma once

#include "../common/run_dump.h"
#include "NowPair.h"

// Stop-and-wait receiver. A repeated record is acknowledged without printing
// it twice, including when the final ACK was lost.
class DumpReceiver {
public:
  bool receive(const uint8_t *data, size_t length) {
    if (!RunDump::is_message(data, length))
      return false;
    last_received_ms = millis();
    const auto type = static_cast<RunDump::Message>(data[3]);
    const uint32_t incoming_session = RunDump::get_u32(data + 4);
    const uint32_t index = RunDump::get_u32(data + 8);
    if (type == RunDump::Message::Status) {
      Serial.print("# dump ");
      Serial.write(data + RunDump::prefix_size, length - RunDump::prefix_size);
      Serial.println();
    } else if (type == RunDump::Message::Begin &&
        length == RunDump::prefix_size + sizeof(RunDump::Header)) {
      RunDump::Header header;
      std::memcpy(&header, data + RunDump::prefix_size, sizeof(header));
      if (header.magic_value != RunDump::magic ||
          header.format_version != RunDump::version ||
          header.record_size != sizeof(RunDump::Record) ||
          header.count > RunDump::capacity)
        return true;
      if (!have_session || session != incoming_session) {
        session = incoming_session;
        have_session = true;
        active = true;
        next = 0;
        checksum = UINT32_MAX;
        expected = header;
        Serial.printf("# dump_begin generation=%lu count=%lu overwritten=%lu paused=%lu\n",
                      static_cast<unsigned long>(header.generation),
                      static_cast<unsigned long>(header.count),
                      static_cast<unsigned long>(header.overwritten),
                      static_cast<unsigned long>(header.paused_samples));
        Serial.print(RunDump::csv_header);
      }
      acknowledge(RunDump::begin_index);
    } else if (have_session && session == incoming_session) {
      if (type == RunDump::Message::Record &&
          length == RunDump::prefix_size + sizeof(RunDump::Record)) {
        if (active && index == next && next < expected.count) {
          RunDump::Record record;
          std::memcpy(&record, data + RunDump::prefix_size, sizeof(record));
          checksum = RunDump::crc32(checksum, &record, sizeof(record));
          char line[256];
          RunDump::csv_record(line, sizeof(line), record);
          Serial.print(line);
          ++next;
        }
        if (next > 0 && index == next - 1)
          acknowledge(index);
      } else if (type == RunDump::Message::End &&
                 length == RunDump::prefix_size && next == expected.count) {
        if ((checksum ^ UINT32_MAX) == expected.crc) {
          if (active)
            Serial.println("# dump_end");
          active = false;
          acknowledge(RunDump::end_index);
        } else {
          Serial.println("# dump checksum mismatch; request e to restart");
          active = false;
        }
      }
    }
    return true;
  }

  void update(NowPair &link) {
    if (active && (!link.connected() || millis() - last_received_ms > 3000)) {
      active = false;
      Serial.println("# dump incomplete; request e to restart");
    }
    if (pending_ack && link.sendData(ack, sizeof(ack)))
      pending_ack = false;
  }
  bool receiving() const { return active; }

private:
  void acknowledge(uint32_t index) {
    RunDump::prefix(ack, RunDump::Message::Ack, session, index);
    pending_ack = true;
  }
  bool have_session = false, active = false, pending_ack = false;
  uint32_t session = 0, next = 0, checksum = UINT32_MAX, last_received_ms = 0;
  RunDump::Header expected{};
  uint8_t ack[RunDump::prefix_size];
};
