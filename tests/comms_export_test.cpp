// Exercise production Comms + NowPair with bounded FreeRTOS queues, heartbeat
// contention, telemetry every control pass, and an emulated acknowledging peer.
#include "../brick/src/subsystems/comms.h"
#include <LittleFS.h>
#include <cassert>
#include <iostream>
#include <memory>

uint32_t host_ms = 1000;
bool host_motion = false;
unsigned host_flash_operations = 0;
HostSerial Serial;
HostLittleFS LittleFS;
std::map<std::string, std::vector<uint8_t>> host_files;
const uint8_t remote_mac[6] = {2, 0, 0, 0, 0, 1};
const uint8_t local_mac[6] = {2, 0, 0, 0, 0, 2};
const uint8_t broadcast_mac[6] = {255, 255, 255, 255, 255, 255};
unsigned begins = 0, records = 0, ends = 0, telemetry = 0, replies = 0;
unsigned duplicates = 0;
RunDump::Header expected{};
uint32_t session = 0, checksum = UINT32_MAX;
bool finished = false;

void receive_frame(uint8_t kind, const void *payload, size_t length) {
  uint8_t frame[122] = {kind, static_cast<uint8_t>(length)};
  if (length)
    std::memcpy(frame + 2, payload, length);
  HostRxControl radio{-55, micros()};
  esp_now_recv_info_t info{remote_mac, kind == 1 ? broadcast_mac : local_mac, &radio};
  host_receive(&info, frame, 2 + length);
}

size_t transmit(ESP_NOW_Peer &peer, const uint8_t *frame, size_t length) {
  // Completion may run before send returns, as permitted by NowPair.
  peer.onSent(true);
  if (frame[0] == 2) {
    receive_frame(3, frame + 2, 4); // Reply to the robot's heartbeat probe.
  } else if (frame[0] == 3) {
    ++replies;
  } else if (frame[0] == 4) {
    const uint8_t *data = frame + 2;
    if (!RunDump::is_message(data, frame[1])) {
      ++telemetry;
    } else {
      const auto type = static_cast<RunDump::Message>(data[3]);
      const uint32_t index = RunDump::get_u32(data + 8);
      if (type == RunDump::Message::Begin) {
        if (!begins || session != RunDump::get_u32(data + 4)) {
          ++begins;
          session = RunDump::get_u32(data + 4);
          std::memcpy(&expected, data + RunDump::prefix_size, sizeof(expected));
          assert(RunDump::supported_format(expected));
          records = 0;
          checksum = UINT32_MAX;
          finished = false;
        }
      } else if (type == RunDump::Message::Record) {
        if (index == records) {
          ++records;
          checksum = RunDump::crc32(checksum, data + RunDump::prefix_size, expected.record_size);
        } else {
          assert(index < records);
          ++duplicates;
          if (index != records - 1)
            return length;
        }
      } else if (type == RunDump::Message::End) {
        assert(records == expected.count);
        assert((checksum ^ UINT32_MAX) == expected.crc);
        if (!finished) {
          ++ends;
          finished = true;
        }
      } else {
        return length; // Status packets need no application ACK.
      }
      uint8_t ack[RunDump::prefix_size];
      RunDump::prefix(ack, RunDump::Message::Ack, session, index);
      receive_frame(4, ack, sizeof(ack));
    }
  }
  return length;
}

int main(int argc, char **argv) {
  const uint32_t step_ms = argc > 1 ? std::stoul(argv[1]) : 10;
  std::string output;
  Serial.sink = &output;
  host_transmit = transmit;
  auto robot = std::make_unique<Comms>();
  assert(robot->begin());
  receive_frame(1, nullptr, 0); // Discover/bind the simulated sender.
  TelemetryPacket packet{};
  uint32_t last_remote_probe = host_ms / 50;
  const auto step = [&] {
    host_ms += step_ms;
    if (host_ms / 50 != last_remote_probe) {
      last_remote_probe = host_ms / 50;
      const uint32_t sequence = host_ms;
      receive_frame(2, &sequence, sizeof(sequence));
    }
    robot->update();
    // Preserve the firmware's telemetry-before-recorder call order. The
    // Comms API must keep telemetry from consuming the export's queue slots.
    robot->send(&packet);
    robot->update_recorder(!host_motion);
  };
  for (unsigned i = 0; i < 500; ++i)
    step();
  assert(robot->connected() && telemetry > 0 && replies > 0);
  for (unsigned i = 0; i < RunDump::capacity; ++i)
    robot->get_recorder().add_rssi(-60 - i % 8, i * 1000);
  host_motion = true;
  const auto operations = host_flash_operations;
  robot->get_recorder().request('s');
  for (unsigned i = 0; i < 150; ++i)
    step();
  const auto before_export = telemetry;
  robot->get_recorder().request('e');
  for (unsigned i = 0; i < 12000 && !ends; ++i)
    step();
  std::cout << "step_ms=" << step_ms << " begin=" << begins << " records=" << records << " end=" << ends
            << " telemetry=" << telemetry << " heartbeat replies=" << replies
            << " retransmitted records=" << duplicates << std::endl;
  assert(begins == 1 && records == RunDump::capacity && ends == 1);
  assert(telemetry <= before_export + 8); // Only an existing backlog may drain.
  assert(host_flash_operations == operations);
  // The next loop observes completion and resumes periodic telemetry.
  for (unsigned i = 0; i < 50; ++i)
    step();
  assert(robot->connected());
  assert(telemetry > before_export + 8);

  // Match the reported path as well: persist, experience a flash-duration
  // service gap, reconnect, then export the complete saved file.
  host_motion = false;
  for (unsigned i = 0; i < 500; ++i)
    step();
  assert(!host_files.empty());
  host_ms += 5000;
  for (unsigned i = 0; i < 100; ++i)
    step();
  assert(robot->connected());
  robot->get_recorder().request('e');
  for (unsigned i = 0; i < 12000 && ends < 2; ++i)
    step();
  assert(begins == 2 && records == RunDump::capacity && ends == 2);
  assert(robot->connected());
  std::cout << "saved-file export passed after service gap and reconnection" << std::endl;
}
