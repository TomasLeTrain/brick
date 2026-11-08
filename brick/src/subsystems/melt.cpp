#include "melt.h"
#include <Arduino.h>
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

  // TODO: calc the following:
  // // TODO: calcualted by PID
  base_spin_power = 0;

  // TODO: can flash led also in direction of vector?

  // can account for all delays in the system that result in a skewed movement
  // direction by simply changing the led offset to align with whichever
  // direction the movement is actually going to
  // TODO: better solution could be to figure out the delays to ensure this
  // solution is robust?
  // TODO: move base value to config/ make configurable at runtime?
  float led_offset_pct = 0.5;

  float led_width_micros = rotation_time_micros * config.led_width;
  float led_offset_micros = rotation_time_micros * led_offset_pct;

  float half_led_width_micros = (led_width_micros * 0.5);

  // center leds around offset
  led_start_timestamp = led_offset_micros - half_led_width_micros;
  led_end_timestamp = led_offset_micros + half_led_width_micros;

  Serial.printf("s:%d %d %d %d\n", micros(), rotation_time_micros,
                led_start_timestamp, led_end_timestamp);
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
  // TODO: deadzones
  // TODO: config for magnitude
  // TODO: desaturation
  output.left_power = input.vx - input.turn_control;
  output.right_power = input.vx + input.turn_control;
  output.led_on = true;
}

// TODO: change multipliers to allow int-only math
void Melt::fast_update_melt() {
  // amount of time spent in current rotation, in us
  int32_t rotation_pct_us = micros() - rotation_start_timestamp;

  const float turn_melt_multiplier = 1;
  // TODO: figure out magnitudes and directions
  // increasing rotation_pct_us moves the controlled heading in direction of
  // rotation. decreasing has opposite effect
  rotation_pct_us += input.turn_control * turn_melt_multiplier;

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
    // TODO: move to somewhere else in config
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

  // ccw setup
  // TODO: switch to both positive -> both motors forwards, change math to match
  left_throttle -= linear_power;
  right_throttle += linear_power;

  output.left_power = left_throttle;
  output.right_power = right_throttle;

  bool leds_on = false;

  Serial.printf("b:%d %d %d %d %d\n", micros(), rotation_pct_us,
                rotation_time_micros, led_start_timestamp, led_end_timestamp);

  // displays heading LED at correct location
  if (led_start_timestamp > led_end_timestamp) {
    // range crosses 0, checking if in range must be done with or
    leds_on = rotation_pct_us >= led_start_timestamp ||
              rotation_pct_us <= led_end_timestamp;
  } else {
    leds_on = rotation_pct_us >= led_start_timestamp &&
              rotation_pct_us <= led_end_timestamp;
  }

  // TODO: add more complex led behavior
  if (leds_on) {
    output.led_on = true;
  } else {
    output.led_on = false;
  }
}
