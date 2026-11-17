#include "run_recorder.h"
#include "../../config.h"
#include <LittleFS.h>
#include <esp_system.h>
#include <esp_partition.h>
#include <algorithm>

const char *RunRecorder::path(int slot) {
  return slot == 0 ? "/run0.bin" : "/run1.bin";
}

bool RunRecorder::validate(int slot, RunDump::Header &header) {
  auto file = LittleFS.open(path(slot), FILE_READ);
  if (!file || file.read(reinterpret_cast<uint8_t *>(&header), sizeof(header)) !=
                   sizeof(header) ||
      !RunDump::supported_format(header) ||
      file.size() != sizeof(header) + header.count * header.record_size)
    return false;

  uint8_t buffer[512];
  uint32_t crc = UINT32_MAX;
  while (file.available()) {
    const size_t length = file.read(buffer, sizeof(buffer));
    if (!length)
      return false;
    crc = RunDump::crc32(crc, buffer, length);
  }
  return (crc ^ UINT32_MAX) == header.crc;
}

void RunRecorder::status(const char *text) {
  // Avoid inserting status text in the middle of a partially printed CSV row.
  std::snprintf(status_message, sizeof(status_message), "%s", text);
  status_pending = true;
  if (!serial_export)
    Serial.printf("# dump %s\n", text);
}

void RunRecorder::mount() {
  mounted = false;
  latest_slot = -1;
  if (!esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                ESP_PARTITION_SUBTYPE_ANY, "spiffs")) {
    status("missing spiffs partition; select a Partition Scheme with SPIFFS and upload again");
    return;
  }
  mounted = LittleFS.begin(false);
  if (!mounted) {
    status("spiffs partition found, mount failed; send f to format (erases saved files)");
    return;
  }
  for (int slot = 0; slot < 2; ++slot) {
    RunDump::Header header;
    if (validate(slot, header) &&
        (latest_slot < 0 || int32_t(header.generation - latest.generation) > 0)) {
      latest = header;
      latest_slot = slot;
    }
  }
  status("filesystem ready; s saves recent history");
}

void RunRecorder::begin() {
  mount();
  if (DUMP_BUTTON_PIN >= 0)
    pinMode(DUMP_BUTTON_PIN, INPUT_PULLUP);
}

void RunRecorder::append(const RunDump::Record &record) {
  if (saving) {
    ++paused;
    return;
  }
  records[head] = record;
  head = (head + 1) % RunDump::capacity;
  if (count < RunDump::capacity)
    ++count;
  else
    ++overwritten;
}

void RunRecorder::add_rssi(int rssi_dbm, uint32_t radio_us) {
  RunDump::Record record{};
  record.kind = 1;
  record.timestamp_us = radio_us;
  record.received_us = micros();
  record.rssi_dbm = rssi_dbm;
  append(record);
}

void RunRecorder::add_state(const RunDump::Record &new_state) {
  state = new_state;
  state.kind = 2;
  append(state);
}

void RunRecorder::request(char command) {
  if (saving || serial_export || radio_export) {
    status("busy");
    return;
  }
  switch (command) {
  case 's': start_save(); break;
  case 'p': start_export(false); break;
  case 'e': start_export(true); break;
  case 'f':
    if (state.connected) {
      status("format requires spin switch off");
      break;
    }
    if (!esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                  ESP_PARTITION_SUBTYPE_ANY, "spiffs")) {
      status("missing spiffs partition; select a Partition Scheme with SPIFFS and upload again");
      break;
    }
    status("formatting filesystem; please wait");
    LittleFS.end();
    mounted = false;
    if (LittleFS.format()) {
      mount();
      if (mounted)
        status("format complete; filesystem ready");
    } else {
      status("format failed");
    }
    break;
  default: break;
  }
}

void RunRecorder::start_save() {
  if (!mounted || !count) {
    status("save failed; filesystem unavailable or buffer empty");
    return;
  }
  pending = {RunDump::magic, RunDump::version,
             latest_slot < 0 ? 1u : latest.generation + 1u,
             uint32_t(count), sizeof(RunDump::Record), 0, overwritten, 0};
  save_file = LittleFS.open(path(latest_slot == 0 ? 1 : 0), FILE_WRITE);
  if (!save_file ||
      save_file.write(reinterpret_cast<const uint8_t *>(&pending),
                      sizeof(pending)) != sizeof(pending)) {
    save_file.close();
    status("cannot open snapshot file");
    return;
  }
  checksum = UINT32_MAX;
  saved = paused = 0;
  saving = true;
  status("saving recent history");
}

void RunRecorder::finish_save(bool success) {
  saving = false;
  save_file.close();
  if (success) {
    const int slot = latest_slot == 0 ? 1 : 0;
    RunDump::Header verified;
    success = validate(slot, verified);
    if (success) {
      latest_slot = slot;
      latest = verified;
      head = count = overwritten = 0;
    }
  }
  status(success ? "saved; p prints CSV, e exports through sender"
                 : "save failed; previous valid snapshot retained");
}

void RunRecorder::save_step() {
  // Freeze the ring while copying it, with at most 16 records per loop.
  RunDump::Record block[16];
  const size_t length = std::min(size_t(16), count - saved);
  const size_t first = (head + RunDump::capacity - count) % RunDump::capacity;
  for (size_t i = 0; i < length; ++i)
    block[i] = records[(first + saved + i) % RunDump::capacity];
  const size_t bytes = length * sizeof(block[0]);
  if (save_file.write(reinterpret_cast<const uint8_t *>(block), bytes) != bytes) {
    finish_save(false);
    return;
  }
  checksum = RunDump::crc32(checksum, block, bytes);
  saved += length;
  if (saved != count)
    return;

  pending.crc = checksum ^ UINT32_MAX;
  pending.paused_samples = paused;
  if (!save_file.seek(0) ||
      save_file.write(reinterpret_cast<const uint8_t *>(&pending),
                      sizeof(pending)) != sizeof(pending)) {
    finish_save(false);
    return;
  }
  save_file.flush();
  finish_save(true);
}

void RunRecorder::start_export(bool radio) {
  if (!mounted || latest_slot < 0) {
    status("no saved snapshot");
    return;
  }
  export_file = LittleFS.open(path(latest_slot), FILE_READ);
  if (!export_file || !export_file.seek(sizeof(RunDump::Header))) {
    export_file.close();
    status("cannot read snapshot");
    return;
  }
  export_index = 0;
  serial_position = serial_size = 0;
  radio_export = radio;
  serial_export = !radio;
  if (radio) {
    session = esp_random();
    sending_begin = true;
    sending_end = waiting_ack = false;
    retries = 0;
  } else {
    Serial.printf("# dump_begin generation=%lu count=%lu overwritten=%lu paused=%lu\n",
                  static_cast<unsigned long>(latest.generation),
                  static_cast<unsigned long>(latest.count),
                  static_cast<unsigned long>(latest.overwritten),
                  static_cast<unsigned long>(latest.paused_samples));
    serial_size = std::snprintf(serial_buffer, sizeof(serial_buffer), "%s",
                                RunDump::csv_header);
  }
}

void RunRecorder::serial_step() {
  if (serial_position < serial_size) {
    const size_t length = std::min(serial_size - serial_position,
                                  size_t(std::max(0, Serial.availableForWrite())));
    if (length)
      serial_position += Serial.write(
          reinterpret_cast<const uint8_t *>(serial_buffer + serial_position), length);
    return;
  }

  serial_position = 0;
  if (export_index == latest.count) {
    serial_export = false;
    export_file.close();
    Serial.println("# dump_end");
    return;
  }
  RunDump::Record record;
  uint8_t raw[sizeof(record)];
  if (export_file.read(raw, latest.record_size) != latest.record_size ||
      !RunDump::decode_record(raw, latest, record)) {
    serial_export = false;
    export_file.close();
    status("read failed; export incomplete");
    return;
  }
  serial_size = RunDump::csv_record(serial_buffer, sizeof(serial_buffer), record);
  ++export_index;
}

void RunRecorder::on_message(const uint8_t *data, size_t length) {
  if (!RunDump::is_message(data, length))
    return;
  const auto type = static_cast<RunDump::Message>(data[3]);
  if (type == RunDump::Message::Command && length == RunDump::prefix_size + 1) {
    request(char(data[RunDump::prefix_size]));
    return;
  }
  if (type != RunDump::Message::Ack || length != RunDump::prefix_size ||
      !radio_export || !waiting_ack || RunDump::get_u32(data + 4) != session)
    return;
  const uint32_t expected = sending_begin ? RunDump::begin_index :
                            sending_end ? RunDump::end_index : export_index;
  if (RunDump::get_u32(data + 8) != expected)
    return;
  waiting_ack = false;
  retries = 0;
  if (sending_begin)
    sending_begin = false;
  else if (sending_end) {
    radio_export = false;
    export_file.close();
    status("radio export complete");
  } else
    ++export_index;
}

void RunRecorder::radio_step(NowPair &link) {
  if (!link.connected()) {
    radio_export = false;
    export_file.close();
    status("radio export interrupted; request e to restart");
    return;
  }
  if (waiting_ack) {
    if (millis() - last_send_ms < 200)
      return;
    if (retries >= 10) {
      radio_export = false;
      export_file.close();
      status("radio export timed out");
      return;
    }
  } else {
    sending_end = !sending_begin && export_index == latest.count;
    const auto type = sending_begin ? RunDump::Message::Begin :
                      sending_end ? RunDump::Message::End : RunDump::Message::Record;
    const uint32_t index = sending_begin ? RunDump::begin_index :
                           sending_end ? RunDump::end_index : export_index;
    RunDump::prefix(radio_buffer, type, session, index);
    radio_size = RunDump::prefix_size;
    if (sending_begin) {
      std::memcpy(radio_buffer + radio_size, &latest, sizeof(latest));
      radio_size += sizeof(latest);
    } else if (!sending_end) {
      if (export_file.read(radio_buffer + radio_size, latest.record_size) !=
          latest.record_size) {
        radio_export = false;
        export_file.close();
        status("radio export read failed");
        return;
      }
      radio_size += latest.record_size;
    }
    waiting_ack = true;
    // Permit the first transmission immediately, including after clock wrap.
    last_send_ms = millis() - 200;
  }
  if (link.sendData(radio_buffer, radio_size)) {
    last_send_ms = millis();
    ++retries;
  }
}

void RunRecorder::update(NowPair &link) {
  if (DUMP_BUTTON_PIN >= 0) {
    const bool down = digitalRead(DUMP_BUTTON_PIN) == LOW;
    if (down != button_raw) {
      button_raw = down;
      button_changed_ms = millis();
    }
    if (millis() - button_changed_ms >= 40 && button_down != button_raw) {
      button_down = button_raw;
      if (button_down)
        request('s');
    }
  }
  if (Serial.available()) {
    const char command = Serial.read();
    if (command == 's' || command == 'p' || command == 'e' || command == 'f')
      request(command);
  }
  if (saving)
    save_step();
  if (serial_export)
    serial_step();
  if (radio_export)
    radio_step(link);
  if (status_pending && link.connected()) {
    uint8_t packet[NowPair::MAX_PAYLOAD];
    RunDump::prefix(packet, RunDump::Message::Status, 0, 0);
    const size_t length = std::strlen(status_message);
    std::memcpy(packet + RunDump::prefix_size, status_message, length);
    if (link.sendData(packet, RunDump::prefix_size + length))
      status_pending = false;
  }
}
