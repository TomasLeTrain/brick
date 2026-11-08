#pragma once

#include "config.h"
#include "src/lib/AlfredoDShot.h"
#include "src/subsystems/comms.h"
#include "src/subsystems/led.h"
#include "src/subsystems/melt.h"
#include "src/subsystems/rpm_estimator.h"

enum ConnectionState {
  NetworkDisconnect,
  ControllerDisconnect,
  SpinSwitchOff,
  AccelDisconnect,
  Ready
};

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

  RpmEstimator rpm_estimator;
  Accelerometer accel;

  LED leds;
  Comms comms;

  Melt melt;

  bool arming = true;

  // only applies when not arming
  // should only be true when no issues are present
  ConnectionState connected = NetworkDisconnect;

  bool accel_disconnected = true;

  float throttle = 0.1;
  int command = 0;

  float tank_linear = 0;
  float tank_angular = 0;
};
