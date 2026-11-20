#include "melt.h"
#include "led.h"
#include <Arduino.h>
#include <cmath>
#include <math.h>

// recalculates all values dependent on input
// call after changing any inputs
void Melt::general_update() {
  // (min / rot) * (60 sec / 1 min) * (1'000'000 usecs / 1 sec) = (usec / rot)
  // NOTE: a minimum in rpm is handled here even if it gets handled elsewhere to
  // avoid infinite rotation time
  rotation_time_micros = (60 * 1'000'000) / fmaxf(input.rpm, 10);

  // only considered melting if above min melting rpm
  melting = input.rpm > config.min_melting_rpm;

  //
  // // can move in any direction by finding the target angle, finding the
  // // difference, and moving accordingly. some impl code taken from
  // //
  // https://github.com/TGower/rotation_sensing/blob/585d0a25411233ff8aafb96f5dd6b3d997775a80/receiver/main/main.c#L345
  //
  target_theta = atan2f(input.vy, input.vx);

  // Normalize 0..2PI
  if (target_theta < 0)
    target_theta += 2.0f * M_PI;
  if (target_theta >= 2.0f * M_PI)
    target_theta -= 2.0f * M_PI;

  float turn_control = input.turn_control;

  // turn control deadzone
  if (fabsf(turn_control) < config.melt_turn_control_deadzone) {
    turn_control = 0;
  }

  if (input.auto_drive_type) {
    if (input.target_rpm_control < config.target_rpm_control_deadzone) {
      active_drive_type = DriveType::Tank;
    } else {
      active_drive_type = DriveType::Melting;
    }
  } else {
    active_drive_type = input.manual_drive_type;
  }

  // apply drive specific configs
  if (active_drive_type == DriveType::Melting) {
    // makes ||v|| <= 1
    float vx = input.vx / M_SQRT2;
    float vy = input.vy / M_SQRT2;

    // guaranteed to be 0 <= mag <= 1
    float mag = sqrt(vx * vx + vy * vy);

    if (mag < config.melt_deadzone) {
      mag = 0;
    }

    melt_linear_mag = mag;

    // only spin if above deadzone
    if (input.target_rpm_control >= config.target_rpm_control_deadzone) {
      throtte_pid.setTargetRPM(input.target_rpm_control *
                               config.max_target_rpm);

      base_spin_power = throtte_pid.update(input.rpm);

      base_spin_power =
          std::min(base_spin_power, config.melt_max_throttle_magnitude);
    } else {
      base_spin_power = 0;
    }

    // base_spin_power =
    //     input.target_rpm_control * config.melt_max_throttle_magnitude;

    // Preserve the previous nominal 1 kHz turning gain, but integrate actual
    // elapsed time independently of rotor phase and RSSI corrections.
    turn_rate_rad_s =
        2 * M_PI * 1000 * turn_control * config.turn_magnitude_pct;

    vector_direction_multiplier = input.is_upright ? 1 : -1;

    // led stuff
    const int32_t half_led_heading_width_us =
        (rotation_time_micros * config.heading_led_width) / 2;

    // center around 0
    // led_start_timestamp wraps around since subtracting
    led_heading_start_us = 0 - half_led_heading_width_us;
    led_heading_end_us = 0 + half_led_heading_width_us;

    if (led_heading_start_us < 0) {
      led_heading_start_us += rotation_time_micros;
    }
    if (led_heading_end_us > rotation_time_micros) {
      led_heading_end_us -= rotation_time_micros;
    }

    float target_theta_pct = target_theta / (2.f * M_PI);
    float vector_led_offset_us = rotation_time_micros * target_theta_pct;

    float half_led_vector_width_us =
        rotation_time_micros * (config.vector_led_width / 2.f);

    // center around the location of the vector
    led_vector_start_us = vector_led_offset_us - half_led_vector_width_us;
    led_vector_end_us = vector_led_offset_us + half_led_vector_width_us;

    if (led_vector_start_us < 0) {
      led_vector_start_us += rotation_time_micros;
    }
    if (led_vector_end_us > rotation_time_micros) {
      led_vector_end_us -= rotation_time_micros;
    }

    led_vector_magnitude = melt_linear_mag;

    uint8_t *desired_led_nums = led_display.desiredNums();

    // update digits with rpm
    int int_rpm = input.rpm;
    desired_led_nums[0] = (int_rpm / 1000) % 10;
    desired_led_nums[1] = (int_rpm / 100) % 10;
    desired_led_nums[2] = (int_rpm / 10) % 10;
    desired_led_nums[3] = int_rpm % 10;

    // after updating digits do general update
    led_display.general_update();

  } else if (active_drive_type == DriveType::Tank) {
    // deadzones
    float linear = input.vx;
    float angular = input.vy;

    if (fabsf(linear) < config.tank_deadzone)
      linear = 0;
    if (fabsf(angular) < config.tank_deadzone)
      angular = 0;

    linear *= config.tank_linear_max_mag;
    angular *= config.tank_angular_max_mag;

    // assumes both go forwards on positive
    tank_left_power = linear - angular;
    // setup has power reversed on one side
    tank_right_power = -(linear + angular);

    // desaturate
    float max_mag = fmaxf(fabsf(tank_left_power), fabsf(tank_right_power));
    if (max_mag > config.tank_max_motor_mag) {
      float multiplier = config.tank_max_motor_mag / max_mag;
      tank_left_power *= multiplier;
      tank_right_power *= multiplier;
    }
  }
}

// updates fast loop params for how to move motors
void Melt::fast_update() {
  const float theta = heading.render(
      micros(),
      active_drive_type == DriveType::Melting ? turn_rate_rad_s.load() : 0);
  if (active_drive_type == DriveType::Melting) {
    fast_update_melt(theta);
  } else if (active_drive_type == DriveType::Tank) {
    fast_update_tank();
  }
}

void Melt::fast_update_tank() {
  output.left_power = tank_left_power;
  output.right_power = tank_right_power;

  // fill_color(255, 255, 0, output.led_red, output.led_green, output.led_blue);
  if (input.is_upright) {
    // fill_color(255, 255, 255, red, green, blue);
    fill_color(255, 255, 255, output.led_red, output.led_green,
               output.led_blue);
  } else {
    fill_color(255, 255, 0, output.led_red, output.led_green, output.led_blue);
  }
}

// TODO: change multipliers to allow int-only math
void Melt::fast_update_melt(float current_theta) {
  const int32_t rotation_pct_us =
      current_theta / (2 * M_PI) * rotation_time_micros;

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

  // NOTE: assumes cannot go below zero meaning on the other direction
  // scales to controller magnitude, independent of max power
  float max_linear_power =
      std::min(config.max_translation_power, base_spin_power) * melt_linear_mag;

  if (config.translation_method == TranslationMethod::Square) {
    // only move when difference inside move_threshold
    // TODO: move to somewhere else in config?
    const float move_threshold = (M_PI / 8.0);
    if (fabsf(diff) < move_threshold) {
      linear_power = max_linear_power;
    } else {
      linear_power = 0;
    }
  } else if (config.translation_method == TranslationMethod::Sine) {
    // Max strength at diff = 0 (cos(0)=1)
    // Zero added at diff = +/- 90 (cos(90)=0)
    // Min strength at diff = +/- 180 (cos(180)=-1)
    linear_power = cosf(diff) * max_linear_power;
  } else if (config.translation_method == TranslationMethod::Linear) {
    // very close linear approximation to sine method
    float factor = 1.0f - (2.0f * fabsf(diff) / M_PI);
    linear_power = factor * max_linear_power;
  }

  // when upside down linear power goes backwards
  linear_power *= vector_direction_multiplier;

  // assumes both + means bot goes forwards
  left_throttle -= linear_power;
  right_throttle += linear_power;

  output.left_power = left_throttle;
  output.right_power = right_throttle;

  led_function(rotation_pct_us, output.led_red, output.led_green,
               output.led_blue);
}

void Melt::led_function(int32_t rotation_pct_us, uint8_t red[], uint8_t green[],
                        uint8_t blue[]) {

  // if not actually moving, just display a solid color
  if (!melting) {
    fill_color(255, 255, 255, red, green, blue);
  }

  // offset for all led functions, should center so that
  // rotation_pct_us = 0 is the direction of movement if moving directly
  // forwards
  // NOTE: most likely negative
  // consider some command sent at t = 0, and that command actuates after delays
  // at t = 1. We want the leds at t = 1 to display the vector that would have
  // been displayed at t = 0, so we shift back time to make it appear as if
  // those dealys dont exist
  rotation_pct_us += rotation_time_micros * input.led_shift_amount;

  if (rotation_pct_us > rotation_time_micros) {
    rotation_pct_us -= rotation_time_micros;
  }
  if (rotation_pct_us < 0) {
    rotation_pct_us += rotation_time_micros;
  }

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
    int num_vector_leds_on = led_vector_magnitude * 8;
    color_first_n(255, 255, 0, red, green, blue, num_vector_leds_on);
  } else if (heading_led_on) {
    // need to calculuate pct in current range
    int32_t curr_led_heading_start_us = led_heading_start_us;
    int32_t curr_led_heading_end_us = curr_led_heading_end_us;
    int32_t curr_rotation_pct_us = rotation_pct_us;

    // make sure [start,end] range is continuous
    if (curr_led_heading_start_us > curr_led_heading_end_us) {
      curr_led_heading_end_us += rotation_time_micros;
      if (curr_rotation_pct_us >= 0)
        curr_rotation_pct_us += rotation_time_micros;
    }

    float width_us = curr_led_heading_end_us - curr_led_heading_start_us;
    float pct = (curr_rotation_pct_us - curr_led_heading_start_us) / width_us;
    led_display.fast_update(pct, red, green, blue);
    // done for testing
    // if (input.is_upright) {
    //   fill_color(0, 0, 255, red, green, blue);
    // } else {
    //   fill_color(255, 0, 0, red, green, blue);
    // }
  } else {
    fill_color(0, 0, 0, red, green, blue);
  }
}
