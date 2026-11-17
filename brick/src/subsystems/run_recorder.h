#pragma once

#include "../common/run_dump.h"
#include "../lib/NowPair.h"
#include <FS.h>
#include <atomic>

class RunRecorder {
public:
  void begin();
  void add_rssi(int rssi_dbm, uint32_t radio_us);
  void add_state(const RunDump::Record &state);
  void request(char command);
  void on_message(const uint8_t *data, size_t length);
  void update(NowPair &link);
  bool printing() const { return serial_export.load(); }

private:
  void append(const RunDump::Record &record);
  bool validate(int slot, RunDump::Header &header);
  void mount();
  void start_save();
  void save_step();
  void start_export(bool radio);
  void serial_step();
  void radio_step(NowPair &link);
  void status(const char *text);
  void finish_save(bool success);
  static const char *path(int slot);

  RunDump::Record records[RunDump::capacity];
  RunDump::Record state{};
  size_t head = 0, count = 0, saved = 0;
  uint32_t overwritten = 0, paused = 0, checksum = UINT32_MAX;
  bool mounted = false, saving = false, radio_export = false;
  std::atomic<bool> serial_export{false};
  int latest_slot = -1;
  RunDump::Header latest{}, pending{};
  fs::File save_file, export_file;
  uint32_t export_index = 0, session = 0, last_send_ms = 0;
  bool waiting_ack = false, sending_begin = false, sending_end = false;
  char serial_buffer[512];
  size_t serial_size = 0, serial_position = 0;
  uint8_t radio_buffer[NowPair::MAX_PAYLOAD];
  size_t radio_size = 0;
  unsigned retries = 0;
  bool button_down = false;
  uint32_t button_changed_ms = 0;
  bool button_raw = false;
  bool status_pending = false;
  char status_message[100]{};
};
