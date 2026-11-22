#include "../brick/src/subsystems/boot_arming.h"
#include <cassert>
#include <initializer_list>
#include <type_traits>

int main() {
  static_assert(std::is_trivial<RetainedArming>::value);
  static_assert(sizeof(RetainedArming) == 12);
  constexpr uint32_t firmware = 0x12345678;
  RetainedArming state{};
  assert(!BootArming::consume(state, ESP_RST_SW, firmware));

  for (auto reason : {ESP_RST_SW, ESP_RST_PANIC, ESP_RST_INT_WDT,
                     ESP_RST_TASK_WDT, ESP_RST_WDT, ESP_RST_BROWNOUT,
                     ESP_RST_PWR_GLITCH, ESP_RST_CPU_LOCKUP}) {
    BootArming::complete(state, firmware);
    assert(BootArming::consume(state, reason, firmware));
    // Reset during recovery/peripheral initialization requires full arming.
    assert(!BootArming::consume(state, reason, firmware));
  }
  for (auto reason : {ESP_RST_UNKNOWN, ESP_RST_POWERON, ESP_RST_EXT,
                     ESP_RST_DEEPSLEEP, ESP_RST_SDIO, ESP_RST_USB,
                     ESP_RST_JTAG, ESP_RST_EFUSE}) {
    BootArming::complete(state, firmware);
    assert(!BootArming::consume(state, reason, firmware));
    assert(!BootArming::consume(state, ESP_RST_SW, firmware));
  }
  // A new firmware image, corrupt retention or an interrupted write cannot
  // authorize skipping arming. A later completed boot can recover again.
  BootArming::complete(state, firmware);
  assert(!BootArming::consume(state, ESP_RST_SW, firmware + 1));
  for (int field = 0; field < 3; ++field) {
    for (int bit = 0; bit < 32; ++bit) {
      BootArming::complete(state, firmware);
      if (field == 0) state.marker ^= uint32_t(1) << bit;
      if (field == 1) state.firmware ^= uint32_t(1) << bit;
      if (field == 2) state.check ^= uint32_t(1) << bit;
      assert(!BootArming::consume(state, ESP_RST_SW, firmware));
    }
  }
  BootArming::complete(state, firmware);
  state.marker = 0;
  assert(!BootArming::consume(state, ESP_RST_PANIC, firmware));
  BootArming::complete(state, firmware);
  assert(BootArming::consume(state, ESP_RST_PANIC, firmware));
  BootArming::complete(state, firmware);
  assert(BootArming::consume(state, ESP_RST_PANIC, firmware));
}
