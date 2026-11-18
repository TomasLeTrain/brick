#include "../brick/src/subsystems/run_recorder.h"
#include "../sender_test/DumpReceiver.h"
#include <LittleFS.h>
#include <cassert>
#include <map>
#include <memory>
#include <set>

uint32_t host_ms = 1000;
bool host_motion = false;
unsigned host_flash_operations = 0;
HostSerial Serial;
HostLittleFS LittleFS;
std::map<std::string, std::vector<uint8_t>> host_files;
using Packets = std::vector<std::vector<uint8_t>>;
std::map<const NowPair *, Packets> messages;
std::set<const NowPair *> blocked_sends;

// Only the public queue boundary is mocked. Recorder and dump receiver logic
// compile unchanged against the same NowPair declarations used by firmware.
NowPair::Broadcaster::Broadcaster(NowPair &owner, uint8_t channel)
    : ESP_NOW_Peer(nullptr, channel), _owner(owner) {}
NowPair::NowPair(Role role, uint8_t channel, uint32_t heartbeat, uint32_t timeout)
    : _role(role), _channel(channel), _heartbeatMs(heartbeat), _timeoutMs(timeout),
      _broadcaster(*this, channel) { _connected = true; }
void NowPair::onSent(bool) {}
bool NowPair::sendData(const void *data, size_t length) {
  if (blocked_sends.count(this))
    return false;
  const auto *bytes = static_cast<const uint8_t *>(data);
  messages[this].emplace_back(bytes, bytes + length);
  return true;
}

int main() {
  NowPair robot(NowPair::Role::Slave), sender(NowPair::Role::Master);
  std::string robot_text, sender_text;
  Serial.sink = &robot_text;
  auto recorder = std::make_unique<RunRecorder>();
  recorder->begin();
  for (unsigned i = 0; i < 400; ++i)
    recorder->add_rssi(-60 - i % 8, i * 1000);
  unsigned operations = host_flash_operations;
  host_motion = true;
  recorder->request('s');
  assert(host_flash_operations == operations);
  // An unresponsive serial host must not block capture or radio servicing.
  Serial.room = 0;
  for (unsigned i = 0; i < 400; ++i) {
    host_ms += 10;
    recorder->add_rssi(-80, 400000 + i * 1000);
    recorder->update(robot, false);
  }
  assert(host_flash_operations == operations && host_files.empty());

  // Export the frozen pre-trigger snapshot directly from RAM while running,
  // including packet/ACK losses and duplicate retransmissions.
  Serial.room = 64;
  recorder->request('e');
  DumpReceiver receiver;
  std::set<uint32_t> dropped_records, dropped_acks;
  for (unsigned i = 0; i < 4000; ++i) {
    host_ms += 10;
    Serial.room = i >= 300 && i < 350 ? 0 : 64;
    Serial.sink = &robot_text;
    recorder->update(robot, false);
    auto outgoing = std::move(messages[&robot]);
    messages[&robot].clear();
    for (const auto &packet : outgoing) {
      const uint32_t index = RunDump::get_u32(packet.data() + 8);
      if (packet[3] == uint8_t(RunDump::Message::Record) &&
          index % 11 == 0 && dropped_records.insert(index).second)
        continue;
      receiver.receive(packet.data(), packet.size());
    }
    Serial.sink = &sender_text;
    receiver.update(sender);
    auto acknowledgements = std::move(messages[&sender]);
    messages[&sender].clear();
    for (const auto &packet : acknowledgements) {
      const uint32_t index = RunDump::get_u32(packet.data() + 8);
      if (index % 7 == 0 && dropped_acks.insert(index).second)
        continue;
      recorder->on_message(packet.data(), packet.size());
    }
  }
  assert(sender_text.find("count=400") != std::string::npos);
  assert(sender_text.find("# dump_end") != std::string::npos);
  assert(!receiver.receiving());
  const auto begin = sender_text.find("# dump_begin");
  const auto end = sender_text.find("# dump_end\n", begin) + 11;
  assert(std::count(sender_text.begin() + begin, sender_text.begin() + end, '\n') == 403);
  assert(host_flash_operations == operations);

  // Persist automatically after actual motor outputs stop. A new drive
  // command pauses an incomplete write before the next filesystem operation.
  Serial.sink = &robot_text;
  host_motion = false;
  for (unsigned i = 0; i < 25; ++i) { host_ms += 10; recorder->update(robot, true); }
  assert(!host_files.empty());
  operations = host_flash_operations;
  host_motion = true;
  for (unsigned i = 0; i < 50; ++i) { host_ms += 10; recorder->update(robot, false); }
  assert(host_flash_operations == operations);
  host_motion = false;
  for (unsigned i = 0; i < 100; ++i) { host_ms += 10; recorder->update(robot, true); }
  const auto &file = host_files.at("/run0.bin");
  RunDump::Header header;
  std::memcpy(&header, file.data(), sizeof(header));
  assert(header.count == 400);
  assert(file.size() == sizeof(header) + 400 * sizeof(RunDump::Record));
  assert((RunDump::crc32(UINT32_MAX, file.data() + sizeof(header),
                        file.size() - sizeof(header)) ^ UINT32_MAX) == header.crc);

  // An interrupted second write must not replace the first complete snapshot.
  for (unsigned i = 0; i < 200; ++i)
    recorder->add_rssi(-70, 800000 + i * 1000);
  recorder->request('s');
  for (unsigned i = 0; i < 15; ++i) {
    host_ms += 10;
    recorder->update(robot, true);
  }
  assert(host_files.count("/run1.bin"));

  // File-backed exports queue while moving, and reboot still selects the file.
  recorder.reset();
  recorder = std::make_unique<RunRecorder>();
  recorder->begin();
  operations = host_flash_operations;
  host_motion = true;
  recorder->request('p');
  for (unsigned i = 0; i < 50; ++i) { host_ms += 10; recorder->update(robot, false); }
  assert(host_flash_operations == operations);
  host_motion = false;
  robot_text.clear();
  for (unsigned i = 0; i < 60; ++i) { host_ms += 10; recorder->update(robot, true); }
  operations = host_flash_operations;
  host_motion = true;
  for (unsigned i = 0; i < 100; ++i) { host_ms += 10; recorder->update(robot, false); }
  assert(host_flash_operations == operations);
  host_motion = false;
  for (unsigned i = 0; i < 2000; ++i) { host_ms += 10; recorder->update(robot, true); }
  assert(robot_text.find("source=flash") != std::string::npos);
  assert(robot_text.find("count=400") != std::string::npos);
  assert(robot_text.find("# dump_end") != std::string::npos);
  const auto file_begin = robot_text.find("# dump_begin");
  const auto file_end = robot_text.find("# dump_end\n", file_begin) + 11;
  assert(std::count(robot_text.begin() + file_begin, robot_text.begin() + file_end, '\n') == 403);

  // A flash-backed radio transfer restarts with a fresh session after motion.
  messages[&robot].clear();
  messages[&sender].clear();
  sender_text.clear();
  DumpReceiver flash_receiver;
  recorder->request('e');
  for (unsigned i = 0; i < 2500; ++i) {
    host_ms += 10;
    host_motion = i >= 60 && i < 110;
    operations = host_flash_operations;
    Serial.sink = &robot_text;
    recorder->update(robot, !host_motion);
    auto outgoing = std::move(messages[&robot]);
    messages[&robot].clear();
    for (const auto &packet : outgoing)
      flash_receiver.receive(packet.data(), packet.size());
    Serial.sink = &sender_text;
    flash_receiver.update(sender);
    auto acknowledgements = std::move(messages[&sender]);
    messages[&sender].clear();
    for (const auto &packet : acknowledgements)
      recorder->on_message(packet.data(), packet.size());
    if (host_motion)
      assert(host_flash_operations == operations);
  }
  assert(sender_text.find("# dump_end") != std::string::npos);
  assert(!flash_receiver.receiving());

  // Radio export must start even if the robot has no USB serial consumer.
  recorder.reset();
  recorder = std::make_unique<RunRecorder>();
  Serial.sink = &robot_text;
  recorder->begin();
  messages[&robot].clear();
  Serial.room = 0;
  recorder->request('e');
  for (unsigned i = 0; i < 100; ++i) {
    host_ms += 10;
    recorder->update(robot, true);
  }
  assert(std::any_of(messages[&robot].begin(), messages[&robot].end(),
      [](const auto &packet) { return packet[3] == uint8_t(RunDump::Message::Begin); }));

  sender_text.clear();
  DumpReceiver usb_independent_receiver;
  for (unsigned i = 0; i < 2500; ++i) {
    host_ms += 10;
    Serial.sink = &robot_text;
    Serial.room = 0;
    recorder->update(robot, true);
    auto outgoing = std::move(messages[&robot]);
    messages[&robot].clear();
    for (const auto &packet : outgoing)
      usb_independent_receiver.receive(packet.data(), packet.size());
    Serial.sink = &sender_text;
    Serial.room = 64;
    usb_independent_receiver.update(sender);
    auto acknowledgements = std::move(messages[&sender]);
    messages[&sender].clear();
    for (const auto &packet : acknowledgements)
      recorder->on_message(packet.data(), packet.size());
  }
  assert(sender_text.find("# dump_end") != std::string::npos);
  assert(!usb_independent_receiver.receiving());

  // Queue rejection used to leave retries == 0 and busy forever.
  Serial.sink = &robot_text;
  blocked_sends.insert(&robot);
  recorder->request('e');
  for (unsigned i = 0; i < 1100; ++i) { host_ms += 10; recorder->update(robot, true); }
  assert(robot_text.find("export made no progress for 10 s") != std::string::npos);
  blocked_sends.erase(&robot);
  messages[&robot].clear();
  recorder->request('e');
  for (unsigned i = 0; i < 10; ++i) { host_ms += 10; recorder->update(robot, true); }
  assert(std::any_of(messages[&robot].begin(), messages[&robot].end(),
      [](const auto &packet) { return packet[3] == uint8_t(RunDump::Message::Begin); }));

  // Cancellation and timeout never close an open flash file during motion.
  operations = host_flash_operations;
  host_motion = true;
  recorder->request('x');
  for (unsigned i = 0; i < 10; ++i) { host_ms += 10; recorder->update(robot, false); }
  assert(host_flash_operations == operations);
  recorder->request('e');
  for (unsigned i = 0; i < 1100; ++i) { host_ms += 10; recorder->update(robot, false); }
  assert(host_flash_operations == operations);
  // Both the cancelled transfer and the blocked request release the busy state.
  host_motion = false;
  messages[&robot].clear();
  recorder->request('e');
  for (unsigned i = 0; i < 30; ++i) { host_ms += 10; recorder->update(robot, true); }
  assert(std::any_of(messages[&robot].begin(), messages[&robot].end(),
      [](const auto &packet) { return packet[3] == uint8_t(RunDump::Message::Begin); }));
  recorder->request('x');
  recorder->update(robot, true);

  // A stalled local serial export also releases busy and permits radio export.
  Serial.room = 0;
  recorder->request('p');
  for (unsigned i = 0; i < 1100; ++i) { host_ms += 10; recorder->update(robot, true); }
  messages[&robot].clear();
  recorder->request('e');
  for (unsigned i = 0; i < 10; ++i) { host_ms += 10; recorder->update(robot, true); }
  assert(std::any_of(messages[&robot].begin(), messages[&robot].end(),
      [](const auto &packet) { return packet[3] == uint8_t(RunDump::Message::Begin); }));

  // Timeout after local CSV has actually started, leaving a partial row.
  recorder->request('x');
  Serial.room = 64;
  for (unsigned i = 0; i < 10; ++i) { host_ms += 10; recorder->update(robot, true); }
  recorder->request('p');
  Serial.room = 7;
  for (unsigned i = 0; i < 100; ++i) { host_ms += 10; recorder->update(robot, true); }
  Serial.room = 0;
  for (unsigned i = 0; i < 1100; ++i) { host_ms += 10; recorder->update(robot, true); }
  messages[&robot].clear();
  recorder->request('e');
  for (unsigned i = 0; i < 10; ++i) { host_ms += 10; recorder->update(robot, true); }
  assert(std::any_of(messages[&robot].begin(), messages[&robot].end(),
      [](const auto &packet) { return packet[3] == uint8_t(RunDump::Message::Begin); }));

}
