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

private:
  // TODO: move to separate class?
  void update_z_buffer(float z_accel);
  float get_average_z_accel();

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

  // used for arming
  int command = 0;

  // buffer of last z accel values to determine if bot is upright or not
  float z_accel_queue[5];
  size_t z_accel_position = 0;
};
