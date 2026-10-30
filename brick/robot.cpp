#include "robot.h"
#include "config.h"
#include <Wire.h>

Robot::Robot() {}

void Robot::begin() {
  left_motor.begin(LEFT_MOTOR_PIN, DSHOT600, false, 14);
  right_motor.begin(RIGHT_MOTOR_PIN, DSHOT600, false, 14);

  Serial.println("init accel");

  // set up I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400'000);

  accel.init(0x18);

  // small wait needed before interacting with accelerometer
  delay(20);

  // sample offset
  // TODO: move to other place/when should this happen?
  for (int i = 0; i < 60; i++) {
    accel.sample_offset();
    delay(1);
  }
  // Serial.println("calibrated accel");
}

// blocking
void Robot::arming_sequence() {
  // perform arming sequence
  command = 0;
  delay(3400);
  arming = false;
}

void Robot::general_update() {

  //  Serial.printf("general update %d\n", micros());

  // leds.leds_on_gradient(thing);
  leds.leds_on_rgb(0, 0, 15);
  // leds.leds_on_no_controller();
  accel.update_reading();

  float x = accel.get_x_accel(false);
  float y = accel.get_y_accel(false);
  float z = accel.get_z_accel(false);

  Serial.printf("x:%.3f,y:%.3f,z:%.3f\n", x, y, z);
}

void Robot::motor_update() {
  if (arming) {
    left_motor.send(command);
  } else {
    left_motor.sendThrottle(throttle);
  }
}
