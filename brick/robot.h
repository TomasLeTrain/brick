#pragma once

#include "config.h"
#include "src/lib/AlfredoDShot.h"
#include "src/subsystems/accelerometer.h"
#include "src/subsystems/comms.h"
#include "src/subsystems/led.h"
#include "src/subsystems/melt.h"
#include "src/subsystems/storage_gate.h"

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

  Accelerometer accel;

  LED leds;
  Comms comms;

  Melt melt;

  std::atomic<bool> arming{true};
  std::atomic<bool> motor_active{true};
  std::atomic<uint32_t> motor_max_gap_us{0};
  StorageGate storage_gate;

  // only applies when not arming
  // should only be true when no issues are present
  std::atomic<ConnectionState> connected{NetworkDisconnect};

  // used for arming
  int command = 0;
};
