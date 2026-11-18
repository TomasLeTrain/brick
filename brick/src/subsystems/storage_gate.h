#pragma once

#include <atomic>
#include <cstdint>

// One general-task request writer and one fast-task acknowledgement writer.
// Odd tokens request parking. A new request gets a new token so a delayed ACK
// from an earlier stop cannot permit flash access after drive has resumed.
class StorageGate {
public:
  void request(bool park) {
    const uint32_t current = epoch.load();
    if (bool(current & 1u) != park)
      epoch.store(current + 1);
  }
  uint32_t token() const { return epoch.load(); }
  static bool requested(uint32_t token) { return token & 1u; }
  bool acknowledged(uint32_t token) const { return parked_epoch.load() == token; }
  void acknowledge(uint32_t token) { parked_epoch.store(token); }
  bool parked() const {
    const uint32_t current = token();
    return requested(current) && acknowledged(current);
  }

private:
  std::atomic<uint32_t> epoch{0}, parked_epoch{0};
};
