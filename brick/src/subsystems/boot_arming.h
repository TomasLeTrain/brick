#pragma once

#include <atomic>
#include <cstdint>
#include <esp_system.h>

// POD in RTC_NOINIT memory: no constructors/default initializers may run on it.
struct RetainedArming {
  uint32_t marker;
  uint32_t firmware;
  uint32_t check;
};

class BootArming {
public:
  static bool recoverable(esp_reset_reason_t reason) {
    switch (reason) {
    case ESP_RST_SW:
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_BROWNOUT:
    case ESP_RST_PWR_GLITCH:
    case ESP_RST_CPU_LOCKUP:
      return true;
    default:
      return false;
    }
  }

  // Consume before initializing peripherals. A second reset during startup
  // must not repeatedly skip a sequence that never completed on this boot.
  static bool consume(volatile RetainedArming &state, esp_reset_reason_t reason,
                      uint32_t firmware) {
    const bool resume = recoverable(reason) && state.marker == armed_marker &&
                        state.firmware == firmware &&
                        state.check == ~(armed_marker ^ firmware);
    state.marker = 0;
    state.check = 0;
    std::atomic_thread_fence(std::memory_order_seq_cst);
    return resume;
  }

  // Call only after the entire sequence and both driver initializations finish.
  // Commit the marker last so an interrupted write is not a completed arming.
  static void complete(volatile RetainedArming &state, uint32_t firmware) {
    state.marker = 0;
    state.firmware = firmware;
    state.check = ~(armed_marker ^ firmware);
    std::atomic_thread_fence(std::memory_order_seq_cst);
    state.marker = armed_marker;
  }

private:
  static constexpr uint32_t armed_marker = 0x42524131; // BRA1, layout revision.
};
