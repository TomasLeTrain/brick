#include "../brick/src/subsystems/storage_gate.h"
#include <cassert>

int main() {
  StorageGate gate;
  assert(!gate.parked());
  gate.request(true);
  const auto first = gate.token();
  assert(StorageGate::requested(first) && !gate.parked());
  gate.acknowledge(first);
  assert(gate.parked());
  // Resume drive, then request another save before the fast task runs. Its
  // old ACK must not authorize this new filesystem operation.
  gate.request(false);
  assert(!gate.parked());
  gate.request(true);
  const auto second = gate.token();
  assert(second != first && !gate.parked());
  gate.acknowledge(first);
  assert(!gate.parked());
  gate.acknowledge(second);
  assert(gate.parked());
  gate.request(false);
  gate.acknowledge(second);
  assert(!gate.parked());
}
