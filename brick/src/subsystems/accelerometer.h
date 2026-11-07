#pragma once

#include "../lib/SparkFun_LIS331_ESP32.h"

class Accelerometer {
public:
  Accelerometer();
  void init(int addr);
  void sample_offset();

  void update_reading();
  float get_x_accel(bool fetch_data = true);
  float get_y_accel(bool fetch_data = true);
  float get_z_accel(bool fetch_data = true);

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
