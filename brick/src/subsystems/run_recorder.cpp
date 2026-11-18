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
  status_serial_pending = true;
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
  if (latest_slot < 0) {
    status("filesystem ready; no saved snapshot, s captures recent history");
  } else {
    char message[100];
    std::snprintf(message, sizeof(message), "filesystem ready; saved generation=%lu records=%lu",
                  static_cast<unsigned long>(latest.generation),
                  static_cast<unsigned long>(latest.count));
    status(message);
  }
}

void RunRecorder::begin() {
  mount();
  if (DUMP_BUTTON_PIN >= 0)
    pinMode(DUMP_BUTTON_PIN, INPUT_PULLUP);
}

void RunRecorder::append(const RunDump::Record &record) {
  if (phase != CapturePhase::Rolling) {
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
  add_rssi(rssi_dbm, radio_us, micros());
}

void RunRecorder::add_rssi(int rssi_dbm, uint32_t radio_us, uint32_t callback_us) {
  RunDump::Record record{};
  record.kind = 1;
  record.timestamp_us = radio_us;
  record.received_us = callback_us;
  record.rssi_dbm = rssi_dbm;
  append(record);
}

void RunRecorder::add_state(const RunDump::Record &new_state) {
  state = new_state;
  state.kind = 2;
  append(state);
}

void RunRecorder::request(char command) {
  if (command == 'x') {
    if (serial_export || radio_export || export_requested)
      stop_export("export cancelled; snapshot retained");
    else
      status("no export active");
    return;
  }
  if (serial_export || radio_export || export_requested) {
    status("export already queued or active; x cancels");
    return;
  }
  switch (command) {
  case 's': start_save(); break;
  case 'p':
  case 'e':
    export_requested = command;
    export_wait = ExportWait::None;
    last_export_progress_ms = millis();
    status("export queued");
    break;
  case 'f':
    if (phase == CapturePhase::Writing) {
      status("busy saving snapshot");
      break;
    }
    format_requested = true;
    status("format queued until motor/LED task idle");
    break;
  default: break;
  }
}

void RunRecorder::format() {
  format_requested = false;
  if (!esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                ESP_PARTITION_SUBTYPE_ANY, "spiffs")) {
    status("missing spiffs partition; select a Partition Scheme with SPIFFS and upload again");
    return;
  }
  LittleFS.end();
  mounted = false;
  if (!LittleFS.format()) {
    status("format failed");
    return;
  }
  mount();
  if (mounted)
    status("format complete; filesystem ready");
}

void RunRecorder::start_save() {
  if (phase != CapturePhase::Rolling) {
    // Keep the existing trigger-time snapshot intact.
    write_failed = false;
    status("snapshot already captured; flash save waits for motor/LED task idle");
    return;
  }
  if (!count) {
    status("capture failed; buffer empty");
    return;
  }
  pending = {RunDump::magic, RunDump::version,
             latest_slot < 0 ? 1u : latest.generation + 1u,
             uint32_t(count), sizeof(RunDump::Record), 0, overwritten, 0};
  checksum = UINT32_MAX;
  saved = paused = 0;
  write_failed = false;
  phase = CapturePhase::Preparing;
  status("captured in RAM; flash save queued until motor/LED task idle");
}

void RunRecorder::prepare_step() {
  // CRC work stays in RAM and is bounded, so capture itself does no filesystem
  // operations in the receive callback or active motor/control loops.
  const size_t length = std::min(size_t(16), count - saved);
  const size_t first = (head + RunDump::capacity - count) % RunDump::capacity;
  for (size_t i = 0; i < length; ++i) {
    const auto &record = records[(first + saved + i) % RunDump::capacity];
    checksum = RunDump::crc32(checksum, &record, sizeof(record));
  }
  saved += length;
  if (saved == count) {
    pending.crc = checksum ^ UINT32_MAX;
    pending.paused_samples = paused;
    saved = 0;
    phase = CapturePhase::Waiting;
    status(mounted ? "RAM snapshot ready; flash save waits for motor/LED task idle"
                   : "RAM snapshot ready; storage unavailable, p/e can export");
  }
}

void RunRecorder::begin_save() {
  save_file = LittleFS.open(path(latest_slot == 0 ? 1 : 0), FILE_WRITE);
  if (!save_file ||
      save_file.write(reinterpret_cast<const uint8_t *>(&pending),
                      sizeof(pending)) != sizeof(pending)) {
    save_file.close();
    write_failed = true;
    status("cannot open snapshot file; RAM capture retained");
    return;
  }
  saved = 0;
  phase = CapturePhase::Writing;
  status("saving captured RAM snapshot to flash");
}

void RunRecorder::finish_save(bool success) {
  phase = CapturePhase::Waiting;
  save_file.close();
  if (success) {
    const int slot = latest_slot == 0 ? 1 : 0;
    RunDump::Header verified;
    success = validate(slot, verified);
    if (success) {
      latest_slot = slot;
      latest = verified;
      head = count = overwritten = 0;
      phase = CapturePhase::Rolling;
    }
  }
  write_failed = !success;
  if (success) {
    char message[100];
    std::snprintf(message, sizeof(message), "saved generation=%lu records=%lu; p prints CSV, e exports through sender",
                  static_cast<unsigned long>(latest.generation),
                  static_cast<unsigned long>(latest.count));
    status(message);
  } else {
    status("save failed; RAM and previous flash snapshot retained, s retries");
  }
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
  saved += length;
  if (saved != count)
    return;

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
  ram_export = phase == CapturePhase::Waiting || phase == CapturePhase::Writing;
  if (ram_export) {
    // No flash access, including open/close, when exporting a live RAM capture.
    pending.paused_samples = paused;
  } else {
    if (!mounted || latest_slot < 0) {
      status("no saved snapshot; s captures recent history");
      return;
    }
    export_file = LittleFS.open(path(latest_slot), FILE_READ);
    if (!export_file || !export_file.seek(sizeof(RunDump::Header))) {
      export_file.close();
      status("cannot read snapshot");
      return;
    }
  }
  const auto &header = ram_export ? pending : latest;
  export_index = 0;
  radio_export = radio;
  serial_export = !radio;
  last_export_progress_ms = millis();
  if (radio) {
    session = esp_random();
    sending_begin = true;
    sending_end = waiting_ack = radio_complete = radio_restart = false;
    retries = 0;
    status("radio export started; CSV prints on sender serial");
  } else {
    serial_position = serial_size = 0;
    serial_finishing = false;
    serial_size = std::snprintf(serial_buffer, sizeof(serial_buffer),
        "# dump_begin generation=%lu count=%lu overwritten=%lu paused=%lu source=%s\n%s",
        static_cast<unsigned long>(header.generation),
        static_cast<unsigned long>(header.count),
        static_cast<unsigned long>(header.overwritten),
        static_cast<unsigned long>(header.paused_samples),
        ram_export ? "RAM" : "flash", RunDump::csv_header);
  }
}

void RunRecorder::stop_export(const char *message) {
  if (!ram_export && (serial_export || radio_export))
    export_close_pending = true;
  // Finish a partially emitted row before printing status, without keeping
  // the transfer busy or touching flash in a receive callback.
  serial_tail_pending = serial_tail_pending || (serial_export && serial_position < serial_size);
  serial_export = radio_export = false;
  export_requested = 0;
  waiting_ack = false;
  export_wait = ExportWait::None;
  status(message);
}

void RunRecorder::serial_step() {
  if (serial_position < serial_size) {
    const size_t length = std::min(serial_size - serial_position,
                                  size_t(std::max(0, Serial.availableForWrite())));
    if (length) {
      const size_t written = Serial.write(
          reinterpret_cast<const uint8_t *>(serial_buffer + serial_position), length);
      serial_position += written;
      if (written)
        last_export_progress_ms = millis();
    }
    return;
  }

  const auto &header = ram_export ? pending : latest;
  serial_position = serial_size = 0;
  if (serial_finishing) {
    if (!ram_export && !storage_idle_now)
      return;
    serial_export = false;
    if (!ram_export)
      export_file.close();
    return;
  }
  if (export_index == header.count) {
    serial_size = std::snprintf(serial_buffer, sizeof(serial_buffer), "# dump_end\n");
    serial_finishing = true;
    return;
  }
  RunDump::Record record;
  if (ram_export) {
    const size_t first = (head + RunDump::capacity - count) % RunDump::capacity;
    record = records[(first + export_index) % RunDump::capacity];
  } else {
    if (!storage_idle_now)
      return;
    uint8_t raw[sizeof(record)];
    if (export_file.read(raw, header.record_size) != header.record_size ||
        !RunDump::decode_record(raw, header, record)) {
      serial_export = false;
      export_file.close();
      status("read failed; export incomplete");
      return;
    }
  }
  serial_size = RunDump::csv_record(serial_buffer, sizeof(serial_buffer), record, header.format_version);
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
  last_export_progress_ms = millis();
  if (sending_begin) {
    sending_begin = false;
    status("sender acknowledged dump header; sending records");
  } else if (sending_end) {
    // ACK callbacks only change RAM state. Closing a file waits for update()
    // to confirm that motor outputs are still disabled.
    radio_complete = true;
  } else {
    ++export_index;
    if (export_index % 128 == 0) {
      char progress[80];
      const auto &header = ram_export ? pending : latest;
      std::snprintf(progress, sizeof(progress), "radio export %lu/%lu records",
                    static_cast<unsigned long>(export_index),
                    static_cast<unsigned long>(header.count));
      status(progress);
    }
  }
}

void RunRecorder::radio_step(NowPair &link) {
  const auto &header = ram_export ? pending : latest;
  if (!ram_export && !storage_idle_now) {
    if (!radio_restart)
      status("radio export paused; waiting for motor/LED task idle; x cancels");
    radio_restart = true;
    return;
  }
  if (radio_complete) {
    radio_export = false;
    if (!ram_export)
      export_file.close();
    status("radio export complete");
    return;
  }
  if (radio_restart) {
    radio_export = false;
    export_file.close();
    export_requested = 'e';
    status("restarting flash export after motor/LED task parked");
    return;
  }
  if (!link.connected()) {
    stop_export("radio export disconnected; snapshot retained, e retries");
    return;
  }
  if (waiting_ack) {
    if (millis() - last_send_ms < 200)
      return;
    if (retries >= 10) {
      stop_export("radio export ACK timed out; check sender firmware/serial, e retries");
      return;
    }
  } else {
    sending_end = !sending_begin && export_index == header.count;
    const auto type = sending_begin ? RunDump::Message::Begin :
                      sending_end ? RunDump::Message::End : RunDump::Message::Record;
    const uint32_t index = sending_begin ? RunDump::begin_index :
                           sending_end ? RunDump::end_index : export_index;
    RunDump::prefix(radio_buffer, type, session, index);
    radio_size = RunDump::prefix_size;
    if (sending_begin) {
      std::memcpy(radio_buffer + radio_size, &header, sizeof(header));
      radio_size += sizeof(header);
    } else if (!sending_end) {
      if (ram_export) {
        const size_t first = (head + RunDump::capacity - count) % RunDump::capacity;
        const auto &record = records[(first + export_index) % RunDump::capacity];
        std::memcpy(radio_buffer + radio_size, &record, sizeof(record));
      } else if (export_file.read(radio_buffer + radio_size, header.record_size) !=
                 header.record_size) {
        radio_export = false;
        export_file.close();
        status("radio export read failed");
        return;
      }
      radio_size += header.record_size;
    }
    waiting_ack = true;
    // Permit the first transmission immediately, including after clock wrap.
    last_send_ms = millis() - 200;
  }
  if (link.sendData(radio_buffer, radio_size)) {
    if (sending_begin && retries == 0)
      status("dump header queued for radio; awaiting sender ACK");
    last_send_ms = millis();
    ++retries;
  }
}

void RunRecorder::print_status() {
  if (serial_export)
    return;
  if (serial_tail_pending) {
    const size_t length = std::min(serial_size - serial_position,
        size_t(std::max(0, Serial.availableForWrite())));
    if (length)
      serial_position += Serial.write(
          reinterpret_cast<const uint8_t *>(serial_buffer + serial_position), length);
    serial_tail_pending = serial_position < serial_size;
    return;
  }
  if (status_position == status_size && status_serial_pending) {
    status_size = std::snprintf(status_serial, sizeof(status_serial),
                                "# dump %s\n", status_message);
    status_position = 0;
    status_serial_pending = false;
    status_output_active.store(true);
  }
  const size_t length = std::min(status_size - status_position,
      size_t(std::max(0, Serial.availableForWrite())));
  if (length)
    status_position += Serial.write(
        reinterpret_cast<const uint8_t *>(status_serial + status_position), length);
  status_output_active.store(status_position < status_size);
}

void RunRecorder::update(NowPair &link, bool storage_idle) {
  if (!storage_idle) {
    idle_seen = false;
    storage_idle_now = false;
  } else {
    if (!idle_seen) {
      idle_since_ms = millis();
      idle_seen = true;
    }
    storage_idle_now = millis() - idle_since_ms >= 200;
  }
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
    const char command = serial_command.feed(Serial.read());
    if (command)
      request(command);
  }
  if ((export_requested || serial_export || radio_export) &&
      millis() - last_export_progress_ms >= export_timeout_ms) {
    stop_export(radio_export && waiting_ack && !radio_restart && retries == 0
        ? "radio TX queue blocked for 10 s; snapshot retained, e retries"
        : "export made no progress for 10 s; snapshot retained, p/e retries");
  }
  if (export_close_pending && storage_idle_now) {
    export_file.close();
    export_close_pending = false;
  }
  if (phase == CapturePhase::Preparing)
    prepare_step();
  print_status();
  // Local CSV must follow complete lines. Radio export never depends on the
  // robot's USB connection or its serial buffer draining.
  if (export_requested) {
    const bool radio = export_requested == 'e';
    const ExportWait wait = phase == CapturePhase::Preparing ? ExportWait::Preparing :
        phase == CapturePhase::Rolling && !storage_idle_now ? ExportWait::Motors :
        radio && !link.connected() ? ExportWait::Link :
        !radio && (status_position != status_size || serial_tail_pending) ? ExportWait::SerialOutput :
        ExportWait::None;
    if (wait != export_wait) {
      export_wait = wait;
      switch (wait) {
      case ExportWait::Preparing: status("export waiting for RAM checksum; x cancels"); break;
      case ExportWait::Motors: status("export waiting for motor/LED task idle for 200 ms; x cancels"); break;
      case ExportWait::Link: status("export waiting for sender ESP-NOW connection; x cancels"); break;
      case ExportWait::SerialOutput: status("export waiting for robot serial output; x cancels"); break;
      case ExportWait::None: break;
      }
    }
    if (wait == ExportWait::None) {
      export_requested = 0;
      start_export(radio);
    }
  }
  if (!serial_export && !radio_export && !export_requested && storage_idle_now) {
    if (format_requested && phase != CapturePhase::Writing)
      format();
    if (phase == CapturePhase::Waiting && mounted && !write_failed)
      begin_save();
    if (phase == CapturePhase::Writing)
      save_step();
  }
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
