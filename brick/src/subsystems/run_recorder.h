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
  void update(NowPair &link, bool storage_idle);
  bool printing() const {
    return serial_export.load() || serial_tail_pending.load() || status_output_active.load();
  }
  // Called on the general task, before it queues best-effort telemetry.
  bool radio_pending() const {
    return radio_export || export_requested == 'e' || status_pending;
  }

private:
  void append(const RunDump::Record &record);
  bool validate(int slot, RunDump::Header &header);
  void mount();
  enum class CapturePhase { Rolling, Preparing, Waiting, Writing };
  void start_save();
  void prepare_step();
  void begin_save();
  void format();
  void print_status();
  void save_step();
  void start_export(bool radio);
  void stop_export(const char *message);
  void serial_step();
  void radio_step(NowPair &link);
  void status(const char *text);
  void finish_save(bool success);
  static const char *path(int slot);

  RunDump::Record records[RunDump::capacity];
  RunDump::Record state{};
  size_t head = 0, count = 0, saved = 0;
  uint32_t overwritten = 0, paused = 0, checksum = UINT32_MAX;
  bool mounted = false, radio_export = false, ram_export = false;
  CapturePhase phase = CapturePhase::Rolling;
  bool write_failed = false, format_requested = false, serial_finishing = false;
  char export_requested = 0;
  bool storage_idle_now = false, idle_seen = false;
  uint32_t idle_since_ms = 0;
  std::atomic<bool> serial_export{false}, status_output_active{false};
  int latest_slot = -1;
  RunDump::Header latest{}, pending{};
  fs::File save_file, export_file;
  uint32_t export_index = 0, session = 0, last_send_ms = 0;
  bool waiting_ack = false, sending_begin = false, sending_end = false;
  bool radio_complete = false, radio_restart = false;
  bool export_close_pending = false;
  enum class ExportWait { None, Preparing, Motors, Link, SerialOutput };
  ExportWait export_wait = ExportWait::None;
  uint32_t last_export_progress_ms = 0;
  static constexpr uint32_t export_timeout_ms = 10000;
  std::atomic<bool> serial_tail_pending{false};
  char serial_buffer[768];
  size_t serial_size = 0, serial_position = 0;
  uint8_t radio_buffer[NowPair::MAX_PAYLOAD];
  size_t radio_size = 0;
  unsigned retries = 0;
  bool button_down = false;
  uint32_t button_changed_ms = 0;
  bool button_raw = false;
  bool status_pending = false;
  char status_message[100]{};
  char status_serial[128]{};
  size_t status_size = 0, status_position = 0;
  bool status_serial_pending = false;
};
