#include <math.h>

#include "accelerometer.h"
#include "src/lib/SparkFun_LIS331_ESP32.h"

Accelerometer::Accelerometer() {}

void Accelerometer::init(int addr) {
  lis.setI2CAddr(addr);
  lis.begin(LIS331ESP::USE_I2C);
  lis.setFullScale(LIS331ESP::HIGH_RANGE);
  lis.setODR(LIS331ESP::DR_1000HZ);
  // TODO: check for i2c disconnect and indicate as error?
}

// Assumption: We're only calling this when the bot is at rest, right-side-up,
// in a 1g environment.
void Accelerometer::sample_offset() {
  int16_t x, y, z;
  lis.readAxes(x, y, z);

  float xg = lis.convertToG(400, x);
  float yg = lis.convertToG(400, y);
  float zg = lis.convertToG(400, z);

  sample_count++;
  summed_x_samples += xg;
  x_offset = summed_x_samples / sample_count;
  summed_y_samples += yg;
  y_offset = summed_y_samples / sample_count;
  summed_z_samples += zg;
  z_offset = (summed_z_samples / sample_count) - 1.0;
}

void Accelerometer::update_reading() { lis.readAxes(x, y, z); }

float Accelerometer::get_x_accel(bool fetch_data) {
  if (fetch_data)
    update_reading();

  float xg = lis.convertToG(400, x);
  return xg - x_offset;
}

float Accelerometer::get_y_accel(bool fetch_data) {
  if (fetch_data)
    update_reading();

  float yg = lis.convertToG(400, y);
  return yg - y_offset;
}

float Accelerometer::get_z_accel(bool fetch_data) {
  if (fetch_data)
    update_reading();

  float zg = lis.convertToG(400, z);
  return zg - z_offset;
}
