#pragma once

#include "pid.h"
#include <cstdint>

enum TranslationMethod { Square, Sine, Linear };
enum DriveType { Melting, Tank };

struct MeltInput {
  // estimated rpm
  float rpm;
  // x component of movement vector
  // repurposed for tank if on tank scheme
  float vx;
  // y component of movement vector
  float vy;

  // in range of [0,1]
  float target_rpm_control;

  // turn component of movement
  float turn_control;

  // type of driving being used
  DriveType drive_type;
};

struct MeltOutput {
  // on range [-1,1]
  float left_power;
  float right_power;

  // in format: rgb0
  uint32_t led_color;
};

struct MeltConfig {
  // the absolute max that is applied to motor for translation
  // on range [-1,1]
  float max_translation_power = 0.1;
  TranslationMethod translation_method = TranslationMethod::Linear;

  // how sensitive turning for melt is
  float turn_magnitude_pct = 0.0001;

  float max_target_rpm = 3000;

  float melt_deadzone = 0.1;
  float melt_turn_control_deadzone = 0.1;
  float tank_deadzone = 0.1;

  // max linear/angular powers for tank
  float tank_linear_max_mag = 0.1;
  float tank_angular_max_mag = 0.1;
  // no motor moves faster than this amount in tank
  float tank_max_motor_mag = 0.2;

  float heading_led_width = 0.5;
  float vector_led_width = 0.1f;
};

class Melt {
  using LedFunction = uint32_t (*)(uint32_t rotation_pct,
                                   uint32_t rotation_total_micros);

public:
  // updates fast loop params for how to move motors
  void fast_update();

  PID *get_throtte_pid() { return &throtte_pid; }
  MeltConfig *get_config() { return &config; }
  MeltInput *get_input() { return &input; }
  const MeltOutput *get_output() { return &output; }

  // recalculates all values dependent on input
  // call after changing any inputs
  void general_update();

private:
  // updates output with tank driving scheme
  void fast_update_tank();
  // updates output with melt driving scheme
  void fast_update_melt();

  uint32_t led_function(int32_t rotation_pct_us);

  MeltInput input;
  MeltOutput output;
  MeltConfig config;

  PID throtte_pid = PID(0, 0);

  // below are values calculated by generated update
  uint32_t rotation_start_timestamp = 0;
  uint32_t rotation_time_micros;

  int32_t led_heading_start_us, led_heading_end_us;
  int32_t led_vector_start_us, led_vector_end_us;

  // TODO: calcualted by PID
  float base_spin_power;

  // in range [0,2PI]
  float target_theta;
  float current_theta_multiplier;

  // how agressively to move
  // in range [0,1]
  float melt_linear_mag;

  // in range [-1,1]
  float tank_left_power;
  float tank_right_power;

  int32_t turn_factor_us;
};
