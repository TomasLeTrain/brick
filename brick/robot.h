#pragma once

#include "src/lib/AlfredoDShot.h"
#include "src/subsystems/accelerometer.h"
#include "src/subsystems/led.h"

class Robot {
public:
  Robot();
  void begin();

  void general_update();
  void motor_update();

  // blocking
  void arming_sequence();

  AlfredoDShot left_motor;
  AlfredoDShot right_motor;
  Accelerometer accel;
  LED leds;

  bool arming = true;
  float throttle = 0.1;
  int command = 0;
};
