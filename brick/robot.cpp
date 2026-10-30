#include "robot.h"
#include "config.h"
#include <Wire.h>

Robot::Robot() {}

void Robot::begin() {
  // TODO: check for any begin errors and report with leds
  leds.set_brightness(10);

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

  Serial.println("did accel");

  // NOTE: if comms used within 800 msec if startup then a temporary disconnect can occur
  // shouldn't cause an issue so no delay is added here
  // init comms
  comms.begin();

  Serial.println("init comms");

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
  // Serial.println("doing update");
  comms.update();

  if (comms.connected()) {
    leds.leds_on_rgb(0, 0, 15);
  } else {
    leds.leds_on_no_controller();
  }

  // Serial.println("accel reading");
  accel.update_reading();

  float x = accel.get_x_accel(false);
  float y = accel.get_y_accel(false);
  float z = accel.get_z_accel(false);
  // Serial.println("getting packet");

  // Serial.printf("x:%.3f,y:%.3f,z:%.3f\n", x, y, z);
  ThrottlePacket *throttle_packet = comms.get_throttle_packet();
   
  if(comms.connected()){
   Serial.printf("packet: %d\n",throttle_packet->throttle);
    // Serial.printf("connected: %d %d\n",comms.get_link()->droppedRxPackets(),micros(), throttle_packet->throttle);
  }else{
    Serial.println("disconnected");
  }

  // now send telemetry
  TelemetryPacket telemetry_packet;
  telemetry_packet.a_x  = x;
  telemetry_packet.a_y  = y;
  telemetry_packet.a_z  = z;

  comms.send(&telemetry_packet);
}

void Robot::motor_update() {
  if (arming) {
    left_motor.send(command);
  } else {
    left_motor.sendThrottle(throttle);
  }
}
