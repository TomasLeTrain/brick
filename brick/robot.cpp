#include "robot.h"
#include "config.h"
#include <Wire.h>

Robot::Robot() {}

void Robot::begin() {
  // TODO: check for any begin errors and report with leds
  leds.set_brightness(10);

  rpm_estimator.set_radius(ACCELEROMETER_HARDWARE_RADIUS_CM);
  rpm_estimator.set_alpha(0.5);

  left_motor.begin(LEFT_MOTOR_PIN, DSHOT600, false, 14);
  right_motor.begin(RIGHT_MOTOR_PIN, DSHOT600, false, 14);

  Serial.println("init accel");

  // set up I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400'000);
  // 10 msec timeout should be more than enough
  Wire.setTimeout(10);
  Serial.println("init i2c");

  accel.init(0x18);

  // small wait needed before interacting with accelerometer
  delay(20);

  // try and sample once to see if there is a connection error or not
  accel.sample_offset();

  // only calibrate if accel is properly connected
  // TODO: propaate error with leds/some other way
  if (!accel.has_connection_error()) {
    // sample offset
    // TODO: move to other place/when should this happen?
    for (int i = 0; i < 60; i++) {
      accel.sample_offset();
      delay(1);
    }

    Serial.println("did accel");
  } else {
    Serial.println("failed accel init!");
  }

  // NOTE: if comms used within 800 msec if startup then a temporary disconnect
  // can occur shouldn't cause an issue so no delay is added here init comms
  comms.begin();
  Serial.println("init comms");
}

// blocking
void Robot::arming_sequence() {
  // perform arming sequence
  command = 0;
  delay(3400);
  // send a non-zero command to init blheli one
  command = 60;
  delay(500);
  // then go back to zero again, fully done
  command = 0;
  delay(500);
  arming = false;
}

void Robot::general_update() {
  // Serial.println("doing update");
  comms.update();

  if (comms.connected()) {
    leds.leds_on_rgb(0, 0, 255);
  } else {
    leds.leds_on_no_controller();
  }

  // Serial.println("accel reading");
  // TODO: can read from accel at faster rate, can also only read x/y and read z
  // less often
  accel.update_reading();

  float x = accel.get_x_accel(false);
  float y = accel.get_y_accel(false);
  float z = accel.get_z_accel(false);


  uint16_t raw_x = accel.get_raw_x_accel();
  uint16_t raw_y = accel.get_raw_y_accel();
  uint16_t raw_z = accel.get_raw_z_accel();
  // Serial.println("getting packet");

  rpm_estimator.update(x, y);

  // Serial.printf("x:%.3f,y:%.3f,z:%.3f\n", x, y, z);
  ThrottlePacket *throttle_packet = comms.get_throttle_packet();

  static uint32_t lastMessage = 0;
  int printing_time = 20;

  if (comms.connected() && throttle_packet->connected) {
    // in range of [-0.5,0.5]
    tank_linear = (throttle_packet->right_ud / 1000.0) - 0.5;
    tank_angular = (throttle_packet->right_lr / 1000.0) - 0.5;

    // get in range of [-1,1]
    tank_linear /= 0.5;
    tank_angular /= 0.5;

    const float MAX_TANK_MAG = 0.3;

    // get in range [-MAX_TANK_MAG,MAX_TANK_MAG]
    tank_linear *= MAX_TANK_MAG;
    tank_angular *= MAX_TANK_MAG;

    // Serial.printf("calc: %.2f %.2f\n",tank_linear, tank_angular);
  } else {
    // not connected, power down
    // TODO: have separate powerdown state instead of using power directly
    Serial.println("NOT CONNECTED!");
    tank_angular = 0;
    tank_linear = 0;
  }

  if (comms.connected()) {
    if (throttle_packet->connected)
      leds.set_brightness((throttle_packet->left_ud * 20) / 1000);

    if (uint32_t(millis() - lastMessage) >= printing_time) {
      // Serial.printf("l_lr=%d,l_ud=%d,r_lr=%d,r_ud=%d,kl=%d,kr=%d,c=%d\n",
      //               throttle_packet->left_lr, throttle_packet->left_ud,
      //               throttle_packet->right_lr, throttle_packet->right_ud,
      //               throttle_packet->knob_left, throttle_packet->knob_right,
      //               throttle_packet->connected);
    Serial.printf("x:%1.3f,y:%.3f,z:%.3f\n", x, y, z);
    // Serial.printf("x:%d,y:%d,z:%d\n", raw_x, raw_y, raw_z);

      lastMessage = millis();
      //  Serial.printf("packet: %d\n",throttle_packet->throttle);
      // Serial.printf("connected: %d
      // %d\n",comms.get_link()->droppedRxPackets(),micros(),
      // throttle_packet->throttle);
    }
  } else {
    Serial.println("disconnected");
  }

  // now send telemetry
  TelemetryPacket telemetry_packet;
  telemetry_packet.a_x = raw_x;
  telemetry_packet.a_y = raw_y;
  telemetry_packet.a_z = raw_z;
  telemetry_packet.rpm = rpm_estimator.get_estimated_rpm();

  comms.send(&telemetry_packet);

  // perform every 10 msec
  vTaskDelay(10);
}

void Robot::motor_update() {
  if (arming) {
    Serial.println("arming");
    left_motor.send(command);
    right_motor.send(command);
  } else {
    const float MAX_TANK_MAG = 0.2;

    // float left_power = tank_linear - tank_angular;
    // float right_power = tank_linear + tank_angular;
    // float max_mag = max(left_power, right_power);

    // // saturation compensation
    // if (max_mag > 1) {
    //   left_power /= max_mag;
    //   right_power /= max_mag;
    // }

    // // cap by max mag
    // left_power *= MAX_TANK_MAG;
    // right_power *= MAX_TANK_MAG;
    // Serial.printf("moving with speeds: %.2f %.2f\n",left_power,right_power);

    // only act on positive
    left_motor.sendThrottle(max(tank_linear, 0.0f));
    right_motor.sendThrottle(max(tank_linear, 0.0f));
  }

  // perform every msec
  vTaskDelay(1);
}
