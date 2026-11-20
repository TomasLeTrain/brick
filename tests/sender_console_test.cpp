#include "../sender_test/DumpReceiver.h"
#include "../sender_test/SenderConsole.h"
#include <cassert>
#include <sstream>
uint32_t host_ms = 1000;
HostSerial Serial;
const uint8_t remote[6] = {2, 0, 0, 0, 0, 1}, local[6] = {2, 0, 0, 0, 0, 2};
const uint8_t broadcast[6] = {255, 255, 255, 255, 255, 255};
void incoming(uint8_t kind, const void *payload, size_t n) {
  uint8_t f[122] = {kind, uint8_t(n)};
  if (n)
    std::memcpy(f + 2, payload, n);
  HostRxControl rx{-55, micros()};
  esp_now_recv_info_t info{remote, kind == 1 ? broadcast : local, &rx};
  host_receive(&info, f, n + 2);
}
size_t transmit(ESP_NOW_Peer &peer, const uint8_t *f, size_t n) {
  peer.onSent(true);
  if (f[0] == 2)
    incoming(3, f + 2, 4);
  return n;
}
int main() {
  std::string output;
  Serial.sink = &output;
  Serial.room = 17;
  host_transmit = transmit;
  NowPair link(NowPair::Role::Slave, 6, 50, 250);
  assert(link.begin());
  incoming(1, nullptr, 0);
  for (int i = 0; i < 8; i++) {
    host_ms += 10;
    link.update();
  }
  assert(link.connected());
  SenderConsole console;
  DumpReceiver dump;
  TelemetryPacket p;
  p.rpm = 2175;
  p.rssi_hz = 900;
  console.receive(p);
  console.update(link, false);
  assert(console.printing());
  RunDump::Record record{};
  record.kind = 2;
  record.rssi_rpm = 2175;
  RunDump::Header header{RunDump::magic,
                         RunDump::version,
                         1,
                         1,
                         sizeof(record),
                         RunDump::crc32(UINT32_MAX, &record, sizeof(record)) ^
                             UINT32_MAX,
                         0,
                         0};
  uint8_t message[RunDump::prefix_size + sizeof(record)]{};
  RunDump::prefix(message, RunDump::Message::Begin, 77, 0);
  std::memcpy(message + RunDump::prefix_size, &header, sizeof(header));
  assert(dump.receive(message, RunDump::prefix_size + sizeof(header)));
  const auto step = [&] {
    host_ms++;
    link.update();
    const bool busy = console.printing();
    dump.update(link, !busy);
    console.update(link, dump.receiving() || dump.printing());
  };
  for (int i = 0; i < 200; i++)
    step();
  assert(!dump.printing() && dump.receiving());
  RunDump::prefix(message, RunDump::Message::Record, 77, 0);
  std::memcpy(message + RunDump::prefix_size, &record, sizeof(record));
  assert(dump.receive(message, RunDump::prefix_size + sizeof(record)));
  for (int i = 0; i < 100; i++)
    step();
  RunDump::prefix(message, RunDump::Message::End, 77, 0);
  assert(dump.receive(message, RunDump::prefix_size));
  for (int i = 0; i < 200; i++)
    step();
  assert(output.find("# dump_end\n") != std::string::npos);
  assert(output.find("@T {\"v\":1") == 0);
  const auto begin = output.find("# dump_begin"),
             end = output.find("# dump_end");
  assert(begin != std::string::npos && end > begin);
  assert(output.substr(begin, end - begin).find("@T ") == std::string::npos);
  assert(output.find(std::string(RunDump::csv_header)) != std::string::npos);
  assert(output.find("\"rpm\":2175.00") != std::string::npos);
  assert(output.find("a_x") == std::string::npos);
}
