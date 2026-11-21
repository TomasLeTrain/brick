#include "../brick/src/subsystems/melt.h"
#include <cassert>
#include <cmath>

uint32_t host_ms = 1000;
void fill_color(uint8_t, uint8_t, uint8_t, uint8_t[], uint8_t[], uint8_t[]) {}
void color_first_n(uint8_t, uint8_t, uint8_t, uint8_t[], uint8_t[], uint8_t[],
                   int) {}

int main() {
  Melt melt;
  auto &input = *melt.get_input();
  input = {};
  input.auto_drive_type = true;
  melt.get_throtte_pid()->setFeedforward(0.00015f);

  // A connected, enabled robot can be idle at neutral throttle and sticks.
  melt.general_update();
  assert(!melt.has_motor_command());

  // Zero spin throttle selects tank mode and can still command either wheel.
  input.vx = 0.8f;
  melt.general_update();
  assert(melt.has_motor_command());
  input.vx = 0;
  input.vy = -0.8f;
  melt.general_update();
  assert(melt.has_motor_command());

  input.vy = 0;
  input.target_rpm_control = 0.6f;
  melt.general_update();
  assert(melt.has_motor_command());
  input.target_rpm_control = 0;
  melt.general_update();
  assert(!melt.has_motor_command());

  // Manual melting also releases idle once its spin request is zero.
  input.auto_drive_type = false;
  input.manual_drive_type = DriveType::Melting;
  melt.general_update();
  assert(!melt.has_motor_command());
  input.target_rpm_control = 0.6f;
  melt.general_update();
  assert(melt.has_motor_command());

  // Rotor phase feeds the actual modulation path, not just diagnostics.
  input.rpm = 1500;
  input.is_upright = true;
  input.vx = .8f;
  input.vy = 0;
  melt.general_update();
  melt.set_phase_reference(host_ms * 1000, 0, 1500);
  melt.fast_update();
  assert(melt.get_output()->left_power > melt.get_output()->right_power);
  melt.set_phase_reference(host_ms * 1000, 3.14159265358979323846f, 1500);
  melt.fast_update();
  assert(melt.get_output()->left_power < melt.get_output()->right_power);
  // Flip reverses translation while retaining the same heading angle.
  melt.set_phase_reference(host_ms * 1000, 0, 1500);
  melt.fast_update();
  const float upright_heading = melt.heading_rad();
  input.is_upright = false;
  melt.general_update();
  melt.fast_update();
  assert(melt.get_output()->left_power < melt.get_output()->right_power);
  assert(melt.heading_rad() == upright_heading);

  // Loss of trustworthy RPM suspends translation, not the feedforward spin.
  input.rpm = 0;
  melt.general_update();
  melt.fast_update();
  assert(melt.get_output()->left_power == melt.get_output()->right_power);
  assert(melt.get_output()->left_power > 0);
  // Low-sensitivity companion turns survive the browser's deadzone scaling;
  // the radio keeps its original deadzone for the same small input.
  const auto small_turn = [](bool companion) {
    Melt steering;
    auto &command = *steering.get_input();
    command = {};
    command.manual_drive_type = DriveType::Melting;
    command.target_rpm_control = .6f;
    command.rpm = 1500;
    command.turn_control = .05f;
    command.companion_heading = companion;
    steering.general_update();
    steering.set_phase_reference(host_ms * 1000, 0, 1500);
    steering.fast_update();
    host_ms += 20;
    steering.set_phase_reference(host_ms * 1000, 0, 1500);
    steering.fast_update();
    return std::abs(
        std::remainder(steering.heading_rad(), 6.2831853071795864769f));
  };
  assert(small_turn(false) < .0001f);
  const float fine_turn = small_turn(true);
  assert(fine_turn > .001f && fine_turn < .02f);
}
