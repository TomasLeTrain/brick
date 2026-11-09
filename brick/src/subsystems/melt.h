#pragma once

#include <cstdint>

enum TranslationMethod { Square, Sine, Linear };
enum DriveType { Melting, Tank };

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

  // in format: rgb0
  uint32_t led_color;
};

// TODO: change to all be in config.h?
struct MeltConfig {
  // the absolute max that is applied to motor for translation
  // on range [-1,1]
  float max_translation_power;
  TranslationMethod translation_method;

  // how sensitive turning for melt is
  float turn_magnitude_pct;
};

class Melt {
  using LedFunction = uint32_t (*)(uint32_t rotation_pct,
                                   uint32_t rotation_total_micros);

public:
  // updates fast loop params for how to move motors
  void fast_update();

  void set_led_function(LedFunction fun) { led_function = fun; }

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

  MeltInput input;
  MeltOutput output;
  MeltConfig config;

  LedFunction led_function = nullptr;

  // below are values calculated by generated update
  uint32_t rotation_start_timestamp = 0;
  uint32_t rotation_time_micros;

  // TODO: calcualted by PID
  float base_spin_power;

  float target_theta;
  float current_theta_multiplier;

  int32_t turn_factor_us;
};
