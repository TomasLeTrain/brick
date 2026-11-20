#include "../common/run_dump.h"
#include <cassert>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> cells(const char *line) {
  std::vector<std::string> result;
  std::istringstream stream(line);
  std::string cell;
  while (std::getline(stream, cell, ',')) result.push_back(cell);
  return result;
}

int main() {
  RunDump::Record record{};
  record.kind = 2;
  record.heading_rad = 1;
  record.heading_error_rad = 2;
  record.rpm_age_ms = 3;
  record.control_rpm = 2100;
  record.hint_used = 5; // Hint enabled, phase tracking.
  char output[512];
  RunDump::csv_record(output, sizeof(output), record, 3);
  auto legacy = cells(output);
  assert(legacy[4] == "1" && legacy[5] == "2" && legacy[6] == "3");
  assert(std::isnan(std::stof(legacy[31])) && std::stoi(legacy[35]) == -1);
  RunDump::csv_record(output, sizeof(output), record, 4);
  auto phase = cells(output);
  assert(phase.size() == cells(RunDump::csv_header).size());
  assert(phase[4] == "0" && phase[31] == "1" && phase[32] == "2");
  assert(phase[24] == "1" && phase[34] == "2100" && std::stoi(phase[35]) == 2);
  RunDump::csv_record(output, sizeof(output), record, 5);
  auto heading = cells(output);
  assert(std::isnan(std::stof(heading[31])) && heading[35] == "-1");
  assert(heading[36] == "1" && heading[37] == "2" && heading[38] == "3");
  for (unsigned version = 1; version <= 5; ++version) {
    RunDump::Header header{RunDump::magic, version, 1, 1,
                          version == 1 ? 56u : version == 2 ? 80u : 84u, 0, 0, 0};
    assert(RunDump::supported_format(header));
  }
}
