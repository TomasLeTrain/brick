#include "robot.h"
#include "melty_config.h"

// converts desired throttle from [-1000, 1000] into value for dshot
int perk2dshot(int throttle) {
  if (throttle == 0) {
    return 0;
  }

  if (throttle > 0) {
    return min(throttle, 999) + 48;
  }

  // otherwise, throttle_perk must be less than zero
  return min(throttle * -1, 998) + 1049;
}

Robot::Robot()
    : left_motor(MOTOR_1_PIN, MOTOR_1_RMT),
      right_motor(MOTOR_2_PIN, MOTOR_2_RMT) {}

void Robot::update_loop(robot_status state,
                        spin_control_parameters_t *spin_params,
                        tank_control_parameters_t *tank_params) {
  switch (state) {
  default:
  case NO_CONTROLLER:
    leds.leds_on_no_controller();
    motors_stop();
    break;
  case CONTROLLER_STALE:
    leds.leds_on_controller_stale();
    motors_stop();
    break;
  case READY:
    leds.leds_on_ready();
    drive_tank(tank_params);
    break;
  case SPINNING:
    spin(spin_params);
  }
}

// motion implemented as follows:
// - forwards is determined as the time when the rotation starts
//   - turning left/right simply shifts the time at which the rotation starts
//
// - the robot moves in some vector relative to the forwards heading

void Robot::spin(spin_control_parameters_t *spin_params) {
  long time_spent_this_rotation_us = micros() - rotation_started_at_us;

  // did a full rotation
  if (time_spent_this_rotation_us > spin_params->rotation_interval_us) {
    // update time spent in rotation
    time_spent_this_rotation_us -= spin_params->rotation_interval_us;
    // update when rotation started
    rotation_started_at_us += spin_params->rotation_interval_us;
  }

  double throttle_offset = 0;

  // translation math time - first, how far into this phase of rotation are we?
  long micros_into_phase =
      time_spent_this_rotation_us % (spin_params->rotation_interval_us / 2);
  float phase_progress =
      2.0 * micros_into_phase / (spin_params->rotation_interval_us);

  // What does that mean the sine (approximation) of that distance into the
  // phase is? Using a parabolic approximation of half a sine wave, that goes
  // from Y=0 to 1 and back in the range X=0..1
  float phase_offset_fraction = -4 * phase_progress * (phase_progress - 1);

  throttle_offset =
      (double)(phase_offset_fraction * spin_params->max_throttle_offset);

  int left_throttle = spin_params->throttle_perk - throttle_offset;
  int right_throttle = spin_params->throttle_perk + throttle_offset;

  if (time_spent_this_rotation_us >= spin_params->motor_start_phase_1 &&
      time_spent_this_rotation_us <= spin_params->motor_start_phase_2) {
    left_throttle *= -1;
    right_throttle *= -1;
  }

  move_with_throttles(left_throttle, right_throttle);

  bool leds_on = false;

  // displays heading LED at correct location
  if (spin_params->led_start > spin_params->led_stop) {
    // range crosses 0, checking if in range must be done with or
    leds_on = time_spent_this_rotation_us >= spin_params->led_start ||
              time_spent_this_rotation_us <= spin_params->led_stop;
  } else {
    leds_on = time_spent_this_rotation_us >= spin_params->led_start &&
              time_spent_this_rotation_us <= spin_params->led_stop;
  }

  if (leds_on) {
    leds.leds_on_gradient(spin_params->battery_percent);
  } else {
    leds.leds_off();
  }
}

void Robot::motors_stop() {
  left_motor.sendThrottleValue(0);
  right_motor.sendThrottleValue(0);
}

void Robot::drive_tank(tank_control_parameters_t *params) {
  if (abs(params->translate_forback) > CONTROL_TRANSLATE_DEADZONE ||
      abs(params->turn_lr) > CONTROL_TRANSLATE_DEADZONE) {
    int forback = params->translate_forback * TANK_FORBACK_POWER_SCALE;
    int leftright = params->turn_lr * TANK_TURNING_POWER_SCALE;

    move_with_throttles(forback + leftright, -1 * (forback - leftright));
  } else {
    motors_stop();
  }
}

float Robot::get_z_buffer() { return imu.z_accel_buffer; }

float Robot::get_rpm(int target_rpm) { return imu.get_rpm(target_rpm); }

int Robot::get_battery() { return battery.get_percent(); }

void Robot::trim_accel(bool increase, int target_rpm) {
  imu.trim(increase, target_rpm);
}

float Robot::get_accel_trim(int target_rpm) { return imu.get_trim(target_rpm); }

void Robot::init() {
  imu.init();
  left_motor.begin(DSHOT300);
  right_motor.begin(DSHOT300);
}

// both throttles on range [-1000,1000]
void Robot::move_with_throttles(int left_throttle, int right_throttle) {
  left_motor.sendThrottleValue(perk2dshot(left_throttle));
  right_motor.sendThrottleValue(perk2dshot(right_throttle));
}
