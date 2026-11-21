#include "../common/companion.h"
#include "../common/orientation_feedback.h"
#include <array>
#include <cassert>
#include <limits>

int main() {
  std::array<uint8_t, 128> buffer{};
  ThrottlePacket control{};
  control.session = 123;
  control.sequence = UINT32_MAX;
  control.connected = control.swa = control.swb = true;
  control.inverted = control.companion_active = true;
  control.left_lr = 0;
  control.left_ud = 1000;
  control.right_lr = 777;
  const size_t n = control.serialize(buffer.data());
  ThrottlePacket restored{};
  assert(restored.deserialize(buffer.data(), n));
  assert(restored.session == 123 && restored.sequence == UINT32_MAX);
  assert(restored.inverted && restored.companion_active && restored.swa &&
         restored.connected);
  assert(restored.left_lr == 0 && restored.left_ud == 1000 &&
         restored.right_lr == 777);
  assert(!restored.deserialize(buffer.data(),
                               8)); // Old controls cannot enable new firmware.
  buffer[2] = 1;
  assert(!restored.deserialize(buffer.data(), n));
  buffer[2] = 2;
  buffer[4] = 0x80;
  assert(restored.deserialize(buffer.data(), n));
  assert(restored.orientation_request == 64 && !restored.companion_active);

  TelemetryPacket telemetry{};
  telemetry.rpm = 3276.5f;
  telemetry.target_rpm = 3300;
  telemetry.throttle = .375f;
  telemetry.rssi_dbm = -77;
  telemetry.inverted = telemetry.companion_active = true;
  telemetry.heading_rad = 6.1f;
  telemetry.heading_error_rad = -.07f;
  telemetry.micros_timestamp = UINT32_MAX;
  telemetry.control_age_ms = 87;
  telemetry.tx_failures = UINT32_MAX;
  telemetry.connection_state = 4;
  const auto size = telemetry.serialize(buffer.data());
  TelemetryPacket got;
  assert(got.deserialize(buffer.data(), size));
  assert(got.rpm == telemetry.rpm && got.throttle == telemetry.throttle);
  assert(got.rssi_dbm == -77 && got.inverted && got.companion_active);
  assert(got.heading_rad == telemetry.heading_rad &&
         got.heading_error_rad == telemetry.heading_error_rad);
  assert(got.tx_failures == UINT32_MAX && got.control_age_ms == 87);
  assert(!got.deserialize(buffer.data(),
                          14)); // No acceleration telemetry ambiguity.
  PacketWire::f32(buffer.data() + 8, std::numeric_limits<float>::infinity());
  assert(!got.deserialize(buffer.data(), size));

  ControlFreshness fresh;
  uint32_t now = UINT32_MAX - 50;
  assert(fresh.accept(control, now, now));
  assert(!fresh.accept(control, now + 10, now + 10));
  control.sequence = 0;
  assert(fresh.accept(control, now + 20, now + 20)); // Sequence rollover.
  assert(fresh.fresh(now + 119));
  assert(!fresh.fresh(now + 120)); // Clock rollover too.
  control.sequence = 1;
  assert(!fresh.accept(control, now, now + 150)); // Delayed queue entry.
  control.session = 999;
  assert(!fresh.accept(control, now + 30, now + 30));
  assert(fresh.accept(control, now + 150, now + 150));

  CompanionControl companion;
  control.left_lr = 250;
  control.left_ud = 800;
  control.right_ud = 700;
  assert(companion.receive("@C,1,45,1,-600,1,1", 1000));
  companion.apply(control, 1000);
  assert(control.left_lr == 200 && control.left_ud == 800 &&
         control.right_ud == 700);
  assert(control.inverted && control.companion_active && control.swa);
  assert(!companion.receive("@C,1,45,1,600,0,1", 1010));
  control.left_lr = 250;
  companion.apply(control, 1150);
  assert(control.left_lr == 250 && !control.companion_active &&
         control.inverted);
  assert(companion.receive("@C,1,45,2,1000,-1,0", 1150));
  companion.apply(control, 1150);
  assert(!control.companion_active && control.inverted &&
         control.left_lr == 250);
  assert(companion.receive("@C,1,45,3,1000,0,1", 1160));
  control.connected = false;
  companion.apply(control, 1160);
  assert(!control.inverted && !control.companion_active &&
         control.left_lr == 250);
  for (const char *bad :
       {"@C,1,45,4,1001,0,1", "@C,1,45,4,0,2,1", "@C,1,45,4,0,0,2",
        "@C,1,0,4,0,0,1", "@C,2,45,4,0,0,1", "@C,1,45,4,0,0,1x",
        "@C,1,45,4,0,0,1,1", "@C,1,45,4,0,0", "@C,1,99999999999999,4,0,0,1"})
    assert(!companion.receive(bad, 1170));

  SerialLines lines;
  const char *line = nullptr;
  for (int i = 0; i < 200; ++i)
    assert(!lines.feed('e'));
  assert(!lines.feed('\n'));
  assert(!lines.feed('e'));
  line = lines.feed('\n');
  assert(line && std::strcmp(line, "e") == 0);
  assert(!lines.feed('e'));
  assert(!lines.feed('\0'));
  assert(!lines.feed('\n'));
  // Explicit selections are one-shot USB actions; ordinary lease heartbeats
  // keep their RF request ID. Selecting the same pose again still acknowledges.
  CompanionControl selections;
  ThrottlePacket selected;
  assert(selections.receive("@C,1,1,1,0,0,0", 0));
  selections.apply(selected, 0);
  assert(selected.orientation_request == 1 && !selected.inverted);
  assert(!selections.receive("@C,1,1,1,0,0,0", 1));
  assert(selections.receive("@C,1,1,2,0,-1,0", 2));
  selections.apply(selected, 2);
  assert(selected.orientation_request == 1);
  assert(selections.receive("@C,1,1,3,0,0,0", 3));
  selections.apply(selected, 3);
  assert(selected.orientation_request == 2);
  // SWD is an independent edge-triggered fallback, never a continuous
  // override of the Joy-Con. Startup/reconnect/failsafe frames are not edges.
  CompanionControl radio;
  radio.observe_radio_orientation(true, false);
  radio.observe_radio_orientation(false, true);
  radio.apply(selected, 0);
  assert(!selected.inverted && selected.orientation_request == 0);
  assert(radio.receive("@C,1,1,1,0,1,1", 0));
  radio.observe_radio_orientation(false, true);
  radio.apply(selected, 0);
  assert(selected.inverted && selected.orientation_request == 1);
  radio.observe_radio_orientation(true, false); // Ignore failsafe values.
  radio.observe_radio_orientation(false, true); // Same on reconnection.
  radio.apply(selected, 1);
  assert(selected.inverted && selected.orientation_request == 1);
  radio.observe_radio_orientation(true, true); // Same pose still acknowledges.
  radio.apply(selected, 2);
  assert(selected.inverted && selected.orientation_request == 2);
  radio.observe_radio_orientation(false, true);
  radio.apply(selected, 151); // No computer lease needed for the fallback.
  assert(!selected.inverted && !selected.companion_active &&
         selected.orientation_request == 3);
  assert(radio.receive("@C,1,1,2,0,1,0", 152));
  radio.observe_radio_orientation(false, true);
  radio.apply(selected, 152);
  assert(selected.inverted && selected.orientation_request == 4);
  radio.observe_radio_orientation(true, true);
  radio.observe_radio_orientation(false, true);
  radio.apply(selected, 153);
  assert(!selected.inverted && selected.orientation_request == 6);
  OrientationRequests requests;
  assert(!requests.accept(1, 0, false)); // Startup isn't a button press.
  assert(requests.accept(1, 1, false));
  assert(!requests.accept(
      1, 1, false)); // RF duplicates/reconnect don't replay feedback.
  assert(requests.accept(1, 127, false));
  assert(requests.accept(1, 0, false)); // Seven-bit revision rollover.
  assert(
      !requests.accept(2, 0, false)); // Sender restart alone isn't an action.
  assert(requests.accept(2, 1, true));

  OrientationFlash flash;
  const uint32_t start = UINT32_MAX - 100;
  flash.observe(2, start);
  assert(flash.active(false, start) && !flash.active(true, start));
  flash.observe(2, start +
                       250); // A retained token cannot extend the 300 ms pulse.
  assert(flash.active(false, start + 299));
  assert(!flash.active(false, start + 300));
  flash.observe(5, start + 301);
  assert(flash.active(true, start + 301) && !flash.active(false, start + 301));
  flash.observe(6,
                start + 320); // New upright action replaces the bottom flash.
  assert(flash.active(false, start + 320) && !flash.active(true, start + 320));
  flash.observe(8,
                start + 400); // Reselecting upright starts a new confirmation.
  assert(flash.active(false, start + 699) && !flash.active(false, start + 700));
}
