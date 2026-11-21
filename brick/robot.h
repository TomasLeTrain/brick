#pragma once

#include "config.h"
#include "src/lib/AlfredoDShot.h"
#include "src/subsystems/comms.h"
#include "src/subsystems/led.h"
#include "src/subsystems/melt.h"
#include "src/subsystems/storage_gate.h"

enum ConnectionState {
  NetworkDisconnect = 0,
  ControllerDisconnect = 1,
  SpinSwitchOff = 2,
  // Value 3 was the removed accelerometer fault; preserve dump compatibility.
  Ready = 4
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
  AlfredoDShot left_motor;
  AlfredoDShot right_motor;

  OrientationFlash orientation_flash;
  LED leds;
  LED bottom_leds{BOTTOM_NEOPIXEL_PIN};
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
