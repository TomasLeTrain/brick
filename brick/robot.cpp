#include "robot.h"
#include "config.h"
#include "src/subsystems/led.h"
#include "src/subsystems/melt.h"
#include <Wire.h>

Robot::Robot() {}

void Robot::begin() {
  // TODO: check for any begin errors and report with leds
  leds.set_brightness(30);

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

  PID *pid = melt.get_throtte_pid();
  pid->setKp(1);
  pid->setKi(0.1);
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

bool last_switch_position = false;

void Robot::general_update() {
  comms.update();
  // TODO: can read from accel at faster rate, can also only read x/y and read z
  // less often
  accel.update_reading();
  int32_t accel_measurement_micros = micros();
  accel_disconnected = accel.has_connection_error();

  float x = accel.get_x_accel(false);
  float y = accel.get_y_accel(false);
  float z = accel.get_z_accel(false);
  uint16_t raw_x = accel.get_raw_x_accel();
  uint16_t raw_y = accel.get_raw_y_accel();
  uint16_t raw_z = accel.get_raw_z_accel();

  ThrottlePacket *throttle_packet = comms.get_throttle_packet();

  if (!comms.connected()) {
    Serial.println("network dc");
    connected = ConnectionState::NetworkDisconnect;
  } else if (!throttle_packet->connected) {
    connected = ConnectionState::ControllerDisconnect;
  } else if (accel_disconnected) {
    connected = ConnectionState::AccelDisconnect;
    // TODO: add switch state
  } else {
    connected = ConnectionState::Ready;
  }

  if (connected == ConnectionState::Ready) {
    // leds.set_brightness((throttle_packet->left_ud * 20) / 1000);

    // [-0.5,0.5]
    float radius_changer = (throttle_packet->knob_left / 1000.0) - 0.5;
    // [-1.5,1.5]
    radius_changer *= 3;

    rpm_estimator.set_radius(ACCELEROMETER_HARDWARE_RADIUS_CM + radius_changer);
  }

  rpm_estimator.update(x, y);
  float curr_rpm = rpm_estimator.get_estimated_rpm();

  if (connected == ConnectionState::Ready) {
    // leds.set_brightness((throttle_packet->left_ud * 20) / 1000);

    if (throttle_packet->swa != last_switch_position) {
      Serial.println("dumping");
      comms.dump_rssi();
      last_switch_position = throttle_packet->swa;
    }

    // [-0.5,0.5]
    float radius_changer = (throttle_packet->knob_left / 1000.0) - 0.5;
    // [-1.5,1.5]
    radius_changer *= 3;

    rpm_estimator.set_radius(ACCELEROMETER_HARDWARE_RADIUS_CM + radius_changer);

    // all in range [-0.5, 0.5]
    float vx = (throttle_packet->right_ud / 1000.0) - 0.5;
    float vy = (throttle_packet->right_lr / 1000.0) - 0.5;
    float turn_control = (throttle_packet->left_lr / 1000.0) - 0.5;
    float target_rpm_control = (throttle_packet->left_ud / 1000.0) - 0.5;

    // convert to range [-0.5, 0.5]
    vx *= 2.0;
    vy *= 2.0;
    turn_control *= 2.0;
    target_rpm_control *= 2.0;

    // in range of [-0.5,0.5]
    tank_linear = (throttle_packet->right_ud / 1000.0) - 0.5;
    tank_angular = (throttle_packet->right_lr / 1000.0) - 0.5;

    // get in range of [-1,1]
    tank_linear *= 2.0;
    tank_angular *= 2.0;

    const float MAX_TANK_MAG = 0.3;

    // get in range [-MAX_TANK_MAG, MAX_TANK_MAG]
    tank_linear *= MAX_TANK_MAG;
    tank_angular *= MAX_TANK_MAG;

    MeltInput *melt_input = melt.get_input();

    melt_input->rpm = curr_rpm;
    melt_input->drive_type = DriveType::Melting;

    // all at zero for now
    melt_input->vx = vx;
    // melt_input->vy = vy;
    // melt_input->turn_control = turn_control;
    // melt_input->target_rpm_control = target_rpm_control;
    melt_input->vy = 0;
    melt_input->turn_control = 0;
    melt_input->target_rpm_control = 0;

    melt.general_update();
  } else {
    if (connected == ConnectionState::NetworkDisconnect) {
      Serial.println("no controller thingy");
      leds.leds_on_no_controller();
    } else if (connected == ConnectionState::ControllerDisconnect) {
      leds.leds_on_controller_stale();
    } else if (connected == ConnectionState::AccelDisconnect) {
      leds.leds_on_rgb(0, 255, 0);
    }

    // regardless, don't move at all
    tank_angular = 0;
    tank_linear = 0;
  }

  static uint32_t lastMessage = 0;
  int printing_time = 20;

  if (comms.connected()) {
    if (uint32_t(millis() - lastMessage) >= printing_time) {
      // Serial.printf("l_lr=%d,l_ud=%d,r_lr=%d,r_ud=%d,kl=%d,kr=%d,c=%d\n",
      //               throttle_packet->left_lr, throttle_packet->left_ud,
      //               throttle_packet->right_lr, throttle_packet->right_ud,
      //               throttle_packet->knob_left, throttle_packet->knob_right,
      //               throttle_packet->connected);
      Serial.printf("x:%1.3f,y:%.3f,z:%.3f\n", x, y, z);
      lastMessage = millis();
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
  telemetry_packet.micros_timestamp = accel_measurement_micros;

  comms.send(&telemetry_packet);

  // perform every 10 msec
  vTaskDelay(10);
}

void Robot::motor_update() {
  if (arming) {
    Serial.println("arming");
    left_motor.send(command);
    right_motor.send(command);
  } else if (connected == ConnectionState::Ready) {
    melt.fast_update();
    const MeltOutput *output = melt.get_output();

    uint32_t led_color = output->led_color;

    leds.leds_on_rgb(led_color & 0xff, (led_color >> 8) & 0xff,
                     (led_color >> 16) & 0xff);

    // don't use these values for now, keeping manual throttle for now
    // left_motor.sendThrottle(max(output->left_power, 0.0f));
    // right_motor.sendThrottle(max(output->right_power, 0.0f));
    left_motor.sendThrottle(max(tank_linear, 0.0f));
    right_motor.sendThrottle(max(tank_linear, 0.0f));
  } else {
    // stop the motors unless strictly good for spinning
    left_motor.send(0);
    right_motor.send(0);
  }

  // perform every msec
  vTaskDelay(1);
}
