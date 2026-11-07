#pragma once

#include <cstdint>

enum TranslationMethod { Square, Sine, Linear };
enum DriveType { Melt, Tank };

// TODO: add mutex?

struct MeltInput {
  // estimated rpm
  float rpm;
  // x component of movement vector
  // repurposed for tank if on tank scheme
  float vx;
  // y component of movement vector
  float vy;

  // turn component of movement
  float turn_control;

  // type of driving being used
  DriveType drive_type;
};

struct MeltOutput {
  // on range [-1,1]
  float left_power;
  float right_power;

  bool led_on;
};

// TODO: change to all be in config.h?
struct MeltConfig {
  // the absolute max that is applied to motor for translation
  // on range [-1,1]
  float max_translation_power;
  TranslationMethod translation_method;

  // 0 - no bar, 1 - entire rotation, 0.5 - half the circle, facing heading,
  // etc.
  float led_width;
};

class Melt {
  // updates fast loop params for how to move motors
  void fast_update();

  const MeltInput *get_input() { return &input; }
  const MeltOutput *get_output() { return &output; }

  // recalculates all values dependent on input
  // call after changing any inputs
  void general_update();

private:
  // updates output with tank driving scheme
  void fast_update_tank();
  // updates output with melt driving scheme
  void fast_update_melt();

  MeltInput input;
  MeltOutput output;
  MeltConfig config;

  // below are values calculated by update rpm
  uint32_t rotation_start_timestamp = 0;
  uint32_t rotation_time_micros;

  // TODO: calcualted by PID
  float base_spin_power;

  int32_t led_start_timestamp;
  int32_t led_end_timestamp;

  float target_theta;
  float current_theta_multiplier;
};
