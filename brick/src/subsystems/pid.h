#pragma once
#include <cstdint>

class PID {
public:
  PID(float kp, float ki) : _kp(kp), _ki(ki) {}
  void setTargetRPM(float target_rpm) { _target_rpm = target_rpm; }

  void setKp(float kp) { _kp = kp; }
  void setKi(float ki) { _ki = ki; }

  float update(float curr_rpm);

private:
  float _kp, _ki, _target_rpm, integral;
  int32_t last_timestamp = -1;
};
