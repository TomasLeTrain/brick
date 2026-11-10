#include "melt.h"
#include "led.h"
#include <Arduino.h>
#include <cmath>
#include <math.h>

// recalculates all values dependent on input
// call after changing any inputs
void Melt::general_update() {
  // (min / rot) * (60 sec / 1 min) * (1'000'000 usecs / 1 sec) = (usec / rot)
  // TODO: min rpm should be handled somewhere else?
  rotation_time_micros = (60 * 1'000'000) / fmaxf(input.rpm, 10);
  //
  // // can move in any direction by finding the target angle, finding the
  // // difference, and moving accordingly. some impl code taken from
  // //
  // https://github.com/TGower/rotation_sensing/blob/585d0a25411233ff8aafb96f5dd6b3d997775a80/receiver/main/main.c#L345
  //
  target_theta = atan2f(-input.vy, input.vx) + M_PI_2;
  // Normalize 0..2PI
  if (target_theta < 0)
    target_theta += 2.0f * M_PI;
  if (target_theta >= 2.0f * M_PI)
    target_theta -= 2.0f * M_PI;

  // precomputed
  // current_theta_multiplier * rotation_pct_us = theta
  current_theta_multiplier = (2 * M_PI) / rotation_time_micros;

  // apply drive specific configs
  if (input.drive_type == DriveType::Melting) {
    // makes ||v|| <= 1
    float vx = input.vx / M_SQRT2;
    float vy = input.vy / M_SQRT2;

    // guaranteed to be 0 <= mag <= 1
    float mag = sqrt(vx * vx + vy * vy);

    if (mag < config.melt_deadzone) {
      mag = 0;
    }

    melt_linear_mag = mag;
  } else if (input.drive_type == DriveType::Tank) {
    // deadzones
    float linear = input.vx;
    float angular = input.turn_control;

    if (fabsf(linear) < config.tank_deadzone)
      linear = 0;
    if (fabsf(angular) < config.tank_deadzone)
      angular = 0;

    linear *= config.tank_linear_max_mag;
    angular *= config.tank_angular_max_mag;

    // assumes both go forwards on positive
    tank_left_power = linear - angular;
    tank_right_power = linear + angular;

    // desaturate
    float max_mag = std::max(tank_left_power, tank_right_power);
    if (max_mag > config.tank_max_motor_mag) {
      float multiplier = config.tank_max_motor_mag / max_mag;
      tank_left_power *= multiplier;
      tank_right_power *= multiplier;
    }
  }

  throtte_pid.setTargetRPM(input.target_rpm_control * config.max_target_rpm);
  base_spin_power = throtte_pid.update(input.rpm);

  float turn_control = input.turn_control;

  // turn control deadzone
  if (fabsf(turn_control) < config.melt_turn_control_deadzone) {
    turn_control = 0;
  }

  // TODO: figure out sign
  // negative increases time of rotation (turns in direction opposite of
  // movement), positive decreases time of rotation (turns in direction of
  // movement)
  // TODO: flipping also flips direction of turning?
  turn_factor_us =
      rotation_time_micros * turn_control * config.turn_magnitude_pct;

  const int32_t half_led_heading_width_us =
      (rotation_time_micros * config.heading_led_width) / 2;

  // center around 0
  // led_start_timestamp wraps around since subtracting
  led_heading_start_us = rotation_time_micros - half_led_heading_width_us;
  led_heading_end_us = half_led_heading_width_us;

  float target_theta_pct = target_theta / (2.f * M_PI);
  float vector_led_offset_us = rotation_time_micros * target_theta_pct;

  const int32_t half_led_vector_width_us =
      (rotation_time_micros * config.vector_led_width) / 2;

  // center around the location of the vector
  // led_start_timestamp wraps around since subtracting
  led_vector_start_us = vector_led_offset_us - half_led_vector_width_us;
  led_vector_end_us = vector_led_offset_us + half_led_heading_width_us;
}

// updates fast loop params for how to move motors
void Melt::fast_update() {
  if (input.drive_type == DriveType::Melting) {
    fast_update_melt();
  } else if (input.drive_type == DriveType::Tank) {
    fast_update_tank();
  }
}

void Melt::fast_update_tank() {
  output.left_power = tank_left_power;
  output.right_power = tank_left_power;

  output.led_color = colorToInt(255, 255, 0);
}

// TODO: change multipliers to allow int-only math
void Melt::fast_update_melt() {
  rotation_start_timestamp += turn_factor_us;

  // amount of time spent in current rotation, in us
  int32_t rotation_pct_us = micros() - rotation_start_timestamp;

  // did a full rotation
  if (rotation_pct_us > rotation_time_micros) {
    // update time spent in rotation
    rotation_pct_us -= rotation_time_micros;
    // update when rotation started
    rotation_start_timestamp += rotation_time_micros;
  }

  float current_theta = rotation_pct_us * current_theta_multiplier;

  // Calculate diff in range -PI to PI
  float diff = current_theta - target_theta;

  // Normalize -PI..PI
  while (diff <= -M_PI)
    diff += 2.0f * M_PI;
  while (diff > M_PI)
    diff -= 2.0f * M_PI;

  float left_throttle = base_spin_power;
  float right_throttle = base_spin_power;

  float linear_power = 0;

  if (config.translation_method == TranslationMethod::Square) {
    // only move when difference inside move_threshold
    // TODO: move to somewhere else in config?
    const float move_threshold = (M_PI / 8.0);
    if (fabsf(diff) < move_threshold) {
      linear_power = config.max_translation_power;
    } else {
      linear_power = 0;
    }
  } else if (config.translation_method == TranslationMethod::Sine) {
    // Max strength at diff = 0 (cos(0)=1)
    // Zero added at diff = +/- 90 (cos(90)=0)
    // Min strength at diff = +/- 180 (cos(180)=-1)
    linear_power = cosf(diff) * config.max_translation_power;
  } else if (config.translation_method == TranslationMethod::Linear) {
    // very close linear approximation to sine method
    float factor = 1.0f - (2.0f * fabsf(diff) / M_PI);
    linear_power = factor * config.max_translation_power;
  }

  linear_power *= melt_linear_mag;

  // TODO: ccw setup?
  // assumes both + means bot goes forwards

  left_throttle -= linear_power;
  right_throttle += linear_power;

  output.left_power = left_throttle;
  output.right_power = right_throttle;

  // TODO: allow function access to more state
  output.led_color = led_function(rotation_pct_us);
}

uint32_t Melt::led_function(int32_t rotation_pct_us) {
  // offset for all led functions, should center so that
  // rotation_pct_us = 0 is the direction of movement if moving directly
  // forwards
  // NOTE: most likely negative?
  float process_delay_led_offset_pct = 0.0;
  rotation_pct_us += rotation_time_micros * process_delay_led_offset_pct;

  bool heading_led_on = false;
  bool movement_vec_led_on = false;

  // displays heading LED at correct location
  if (led_heading_start_us > led_heading_end_us) {
    // range crosses 0, checking if in range must be done with or
    heading_led_on = rotation_pct_us >= led_heading_start_us ||
                     rotation_pct_us <= led_heading_end_us;
  } else {
    heading_led_on = rotation_pct_us >= led_heading_start_us &&
                     rotation_pct_us <= led_heading_end_us;
  }

  if (led_vector_start_us > led_vector_end_us) {
    // range crosses 0, checking if in range must be done with or
    movement_vec_led_on = rotation_pct_us >= led_vector_start_us ||
                          rotation_pct_us <= led_vector_end_us;
  } else {
    movement_vec_led_on = rotation_pct_us >= led_vector_start_us &&
                          rotation_pct_us <= led_vector_end_us;
  }

  if (movement_vec_led_on) {
    return colorToInt(255, 0, 0);
  } else if (heading_led_on) {
    return colorToInt(0, 0, 255);
  } else {
    return 0;
  }
}
