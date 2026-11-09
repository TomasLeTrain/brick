#include "pid.h"
#include <Arduino.h>

float PID::update(float curr_rpm) {
  float error = _target_rpm - curr_rpm;

  int32_t now = millis();
  if (last_timestamp == -1)
    last_timestamp = now;
  int32_t dt = now - last_timestamp;
  last_timestamp = now;

  integral += error * dt;

  float result = error * _kp + integral * _ki;

  return result;
}
