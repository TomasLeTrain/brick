#pragma once

#include "../../config.h"
#include "pid.h"
#include "heading_clock.h"
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
  // also repurposed for tank
  float turn_control;

  // amount of shift in the led movement
  float led_shift_amount;

  // drive type used if auto_drive_type is off
  DriveType manual_drive_type;

  // if on, drive type is determined by melt throttle
  // when melt throttle is below deadzone, tank mode is used
  bool auto_drive_type;

  // whether robot is upright
  bool is_upright;
};

struct MeltOutput {
  // on range [-1,1]
  float left_power;
  float right_power;

  // in format: rgb0
  uint32_t led_color;

  uint8_t led_red[NUM_LEDS];
  uint8_t led_green[NUM_LEDS];
  uint8_t led_blue[NUM_LEDS];
};

struct MeltConfig {
  // the absolute max that is applied to motor for translation
  // on range [0, 1]
  float max_translation_power = 0.1;
  TranslationMethod translation_method = TranslationMethod::Sine;

  // how sensitive turning for melt is
  float turn_magnitude_pct = 0.0013;

  float max_target_rpm = 3000;

  // deadzone for translation vector magnitude
  // if magnitude below this value then no translation occurs
  // on range [0, 1]
  float melt_deadzone = 0.1;

  // deadzone for turning
  // on range [0, 1]
  float melt_turn_control_deadzone = 0.1;

  // deadzone for driving/turning in tank
  // on range [0, 1]
  float tank_deadzone = 0.3;

  // max throttle applied to either motor when melting
  // caps out motors to prevent case where they go full speed
  // on range [0, 1]
  float melt_max_throttle_magnitude = 0.6;

  // max linear/angular powers for tank
  float tank_linear_max_mag = 0.03;
  float tank_angular_max_mag = 0.01;
  // no motor moves faster than this amount in tank
  float tank_max_motor_mag = 0.2;

  float heading_led_width = 0.5f;
  float vector_led_width = 0.02f;

  // rpm must reach above this to be considered melting
  float min_melting_rpm = 400;

  // only apply target_rpm_control when its above this threshold
  // on range [0, 1]
  float target_rpm_control_deadzone = 0.1;
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
  void set_phase_reference(uint32_t timestamp_us, float phase_rad, float rpm) {
    heading.publish(timestamp_us, phase_rad, rpm);
  }
  float heading_rad() const { return heading.angle(); }

  // General-loop intent, independent of the motor task's last applied output.
  // Zero RPM request can select tank mode, which may still command movement.
  bool has_motor_command() const {
    return active_drive_type == DriveType::Melting ? base_spin_power != 0 :
        tank_left_power != 0 || tank_right_power != 0;
  }

  // recalculates all values dependent on input
  // call after changing any inputs
  void general_update();

private:
  // updates output with tank driving scheme
  void fast_update_tank();
  // updates output with melt driving scheme
  void fast_update_melt(float current_theta);

  void led_function(int32_t rotation_pct_us, uint8_t red[], uint8_t green[],
                    uint8_t blue[]);

  MeltInput input{};
  MeltOutput output{};
  MeltConfig config;

  PID throtte_pid;

  // below are values calculated by generated update
  HeadingClock heading;
  uint32_t rotation_time_micros = 6000000;

  int32_t led_heading_start_us, led_heading_end_us;
  int32_t led_vector_start_us, led_vector_end_us;

  // in range [0,1]
  float led_vector_magnitude;

  float base_spin_power = 0;

  // in range [0,2PI]
  float target_theta;

  // how agressively to move
  // in range [0,1]
  float melt_linear_mag;

  // in range [-1,1]
  float tank_left_power = 0;
  float tank_right_power = 0;

  std::atomic<float> turn_rate_rad_s{0};

  // -1 or 1, determined by whether robot is upside down or not
  float vector_direction_multiplier;

  // on when robot is spinning at a fast enough speed
  bool melting = false;

  // drive type being used for update, as determined by inputs
  DriveType active_drive_type = DriveType::Tank;
};
