#pragma once
#include <cstdint>

class PID {
public:
  PID() {}
  void setTargetRPM(float target_rpm) {
    _target_rpm = target_rpm;
    // resets integral on movement change
    integral = 0;
  }

  void setFeedforward(float throttle_per_rpm) {
    this->_throttle_per_rpm = throttle_per_rpm;
  }

  void setKp(float kp) { _kp = kp; }
  void setKi(float ki) { _ki = ki; }
  void setMaxIntegral(float max_integral) { _max_integral = max_integral; }

  float update(float curr_rpm);

private:
  float _kp = 0, _ki = 0, _target_rpm = 0, integral = 0;

  // constant relating the relation between throttle speed and
  // derived through system identification (looking at logs)
  float _throttle_per_rpm = 0;

  // caps the maximum the integral can be
  // done as failsafe to prevent the integral from growing without bound and
  // making the roobt spin forever
  float _max_integral = 0;

  // puts a limit on the acceleration of the throttle
  // done since going from 0 to 100 instantly puts the robot into an unstable
  // tilted state
  // TODO: driver could compensate for this?
  // float upwards_slew = 0;
  //
  // // used for upwards slew
  // float last_throttle = 0;

  int32_t last_timestamp = -1;
};
