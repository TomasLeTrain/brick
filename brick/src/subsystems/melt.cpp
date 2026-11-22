#include "melt.h"
#include "led.h"
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <math.h>

float Melt::spin_feedforward(float speed, bool max_throttle) const {
  if (!std::isfinite(speed) || speed < config.target_rpm_control_deadzone)
    return 0;
  speed = std::clamp(speed, 0.f, 1.f);
  return max_throttle
             ? speed
             : std::min(speed * config.max_target_rpm * throtte_pid.feedforward(),
                        config.melt_max_throttle_magnitude);
}

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

  // Companion input is already deadzoned before sensitivity scaling. Applying
  // the radio deadzone again would erase fine inputs at low sensitivity.
  if (!input.companion_heading &&
      fabsf(turn_control) < config.melt_turn_control_deadzone) {
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

      base_spin_power = input.max_throttle
                            ? spin_feedforward(input.target_rpm_control, true)
                            : std::clamp(throtte_pid.update(input.rpm), 0.f,
                                         config.melt_max_throttle_magnitude);
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

    // The solid heading bar is separate from the back-facing RPM display.
    const float bar_width = std::isfinite(config.bar_led_width)
                                ? std::clamp(config.bar_led_width, 0.f, 1.f)
                                : 0;
    led_bar_half_width_us = rotation_time_micros * bar_width / 2;

    // led stuff
    const int32_t half_led_heading_width_us =
        (rotation_time_micros * config.heading_led_width) / 2;

    // for now, center at other side to test readability
    float led_heading_offset_us = rotation_time_micros * 0.5f;
    led_display_start_us = led_heading_offset_us - half_led_heading_width_us;
    led_display_end_us = led_heading_offset_us + half_led_heading_width_us;

    if (led_display_start_us < 0) {
      led_display_start_us += rotation_time_micros;
    }
    if (led_display_end_us > rotation_time_micros) {
      led_display_end_us -= rotation_time_micros;
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
    const int int_rpm =
        std::isfinite(input.rpm) ? int(std::clamp(input.rpm, 0.f, 9999.f)) : 0;
    desired_led_nums[0] = (int_rpm / 1000) % 10;
    desired_led_nums[1] = (int_rpm / 100) % 10;
    desired_led_nums[2] = 0;
    desired_led_nums[3] = 0;

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
  led_function(theta / (2 * M_PI) * rotation_time_micros, output.led_red,
               output.led_green, output.led_blue);
}

void Melt::fast_update_tank() {
  output.left_power = tank_left_power;
  output.right_power = tank_right_power;
}

// TODO: change multipliers to allow int-only math
void Melt::fast_update_melt(float current_theta) {
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
      (melting ? std::min(config.max_translation_power, base_spin_power) *
                     melt_linear_mag
               : 0);

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
  // left_throttle -= linear_power;
  // right_throttle += linear_power;
  //
  left_throttle += linear_power;
  right_throttle -= linear_power;

  output.left_power = std::clamp(left_throttle, -1.f, 1.f);
  output.right_power = std::clamp(right_throttle, -1.f, 1.f);
}

void Melt::led_function(int32_t rotation_pct_us, uint8_t red[], uint8_t green[],
                        uint8_t blue[]) {
  // Use the same latency-shifted angle for all three ranges and the hue wheel.
  rotation_pct_us += rotation_time_micros * input.led_shift_amount;
  rotation_pct_us %= int32_t(rotation_time_micros);
  if (rotation_pct_us < 0)
    rotation_pct_us += rotation_time_micros;

  fill_color(LedPalette::background.red, LedPalette::background.green,
             LedPalette::background.blue, red, green, blue);
  if (active_drive_type == DriveType::Melting && melting) {
    const auto in_range = [rotation_pct_us](int32_t start, int32_t end) {
      return start > end ? rotation_pct_us >= start || rotation_pct_us <= end
                         : rotation_pct_us >= start && rotation_pct_us <= end;
    };
    // Back-to-front layers: heading bar, RPM text, translation vector. Painting
    // only the vector's lit rows leaves the underlying bar/text visible.
    const int32_t distance_to_zero = std::min(
        rotation_pct_us, int32_t(rotation_time_micros) - rotation_pct_us);
    if (led_bar_half_width_us > 0 &&
        distance_to_zero <= led_bar_half_width_us) {
      fill_color(config.bar_led_color.red, config.bar_led_color.green,
                 config.bar_led_color.blue, red, green, blue);
    }
    if (in_range(led_display_start_us, led_display_end_us)) {
      int32_t start = led_display_start_us, end = led_display_end_us;
      int32_t current = rotation_pct_us;
      if (start > end) {
        end += rotation_time_micros;
        if (current < start)
          current += rotation_time_micros;
      }
      if (end > start)
        led_display.fast_update(float(current - start) / (end - start), red,
                                green, blue);
    }
    const int count =
        std::clamp(int(led_vector_magnitude * LedDisplay::data_pixels), 0,
                   LedDisplay::data_pixels);
    if (count > 0 && in_range(led_vector_start_us, led_vector_end_us)) {
      for (int i = 0; i < count; ++i) {
        red[i] = LedPalette::vector.red;
        green[i] = LedPalette::vector.green;
        blue[i] = LedPalette::vector.blue;
      }
    }
  }
  // Always write this last, including idle/tank mode; it can never become a
  // text stroke or a vector/bar row. One revolution traverses the full hue
  // wheel.
  const LedColor wheel =
      LedPalette::hue(float(rotation_pct_us) / rotation_time_micros);
  red[LedDisplay::wheel_pixel] = wheel.red;
  green[LedDisplay::wheel_pixel] = wheel.green;
  blue[LedDisplay::wheel_pixel] = wheel.blue;
  // Physical face palettes are fixed. Only the LED driver acknowledgement
  // temporarily inverts the selected face, independently of motor orientation.
}
