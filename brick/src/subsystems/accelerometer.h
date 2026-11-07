#pragma once

#include "../lib/SparkFun_LIS331_ESP32.h"

class Accelerometer {
public:
  Accelerometer();
  void init(int addr);
  void sample_offset();

  void update_reading();

  // only updates xy - faster reads than if z is included
  void update_xy();
  float get_x_accel(bool fetch_data = true);
  float get_y_accel(bool fetch_data = true);
  float get_z_accel(bool fetch_data = true);

  // converts back into int representation
  int16_t get_raw_x_accel(bool fetch_data = true) {
    float calc_x = get_x_accel(fetch_data);
    return (calc_x * 2047.0f) / 400.0f;
  }
  int16_t get_raw_y_accel(bool fetch_data = true) {
    float calc_y = get_y_accel(fetch_data);
    return (calc_y * 2047.0f) / 400.0f;
  }

  int16_t get_raw_z_accel(bool fetch_data = true) {
    float calc_z = get_z_accel(fetch_data);
    return (calc_z * 2047.0f) / 400.0f;
  }

  bool has_connection_error() { return lis.hasConnectionError(); };

private:
  LIS331ESP lis;

  int16_t x, y, z;

  int sample_count;
  float summed_x_samples;
  float summed_y_samples;
  float summed_z_samples;
  float x_offset;
  float y_offset;
  float z_offset;
};
