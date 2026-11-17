#include "pid.h"
#include <Arduino.h>
#include <cmath>

float sgn(float x) { return x < 0 ? -1 : 1; }

float PID::update(float curr_rpm) {
  float error = _target_rpm - curr_rpm;

  int32_t now = millis();
  if (last_timestamp == -1)
    last_timestamp = now;
  int32_t dt = now - last_timestamp;
  last_timestamp = now;

  integral += error * dt;
  // cap integral
  if (fabsf(integral) > _max_integral) {
    integral = _max_integral * sgn(integral);
  }

  float feedforward = _throttle_per_rpm * _target_rpm;
  float result = feedforward + error * _kp + integral * _ki;

  return result;
}
