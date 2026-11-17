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

  // NOTE: if comms used within 800 msec if startup then a temporary
  // disconnect can occur shouldn't cause an issue so no delay is added here
  // init comms
  comms.begin();
  Serial.println("init comms");

  PID *pid = melt.get_throtte_pid();
  pid->setKp(KP);
  pid->setKi(KI);
  pid->setFeedforward(RPM_THROTTLE_FEEDFORWARD);
  pid->setMaxIntegral(MAX_INTEGRAL);

  // init z accel queue
  for (int i = 0; i < sizeof(z_accel_queue) / sizeof(z_accel_queue[0]); i++) {
    z_accel_queue[i] = 0.0;
  }
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

  // update z accel buffer
  update_z_buffer(z);

  ThrottlePacket *throttle_packet = comms.get_throttle_packet();

  if (!comms.connected()) {
    Serial.println("network dc");
    connected = ConnectionState::NetworkDisconnect;
  } else if (!throttle_packet->connected) {
    connected = ConnectionState::ControllerDisconnect;
  } else if (accel_disconnected) {
    connected = ConnectionState::AccelDisconnect;
  } else if (!throttle_packet->swa) {
    connected = ConnectionState::SpinSwitchOff;
  } else {
    connected = ConnectionState::Ready;
  }

  if (connected == ConnectionState::Ready) {
    // [-0.5,0.5]
    float radius_changer = (throttle_packet->knob_left / 1000.0) - 0.5;
    // [-0.5,0.5]
    radius_changer *= 0.5 * 2;

    rpm_estimator.set_radius(ACCELEROMETER_HARDWARE_RADIUS_CM + radius_changer);
  }

  rpm_estimator.update(x, y);
  float curr_rpm = rpm_estimator.get_estimated_rpm();

  if (connected == ConnectionState::Ready) {
    // all in range [-0.5, 0.5]
    float vx = (throttle_packet->right_ud / 1000.0) - 0.5;
    float vy = (throttle_packet->right_lr / 1000.0) - 0.5;
    float turn_control = (throttle_packet->left_lr / 1000.0) - 0.5;

    // in range [0,1]
    float target_rpm_control = (throttle_packet->left_ud / 1000.0);
    // float led_shift_amount = (throttle_packet->knob_right / 1000.0);

    // convert to range [-1, 1]
    vx *= 2.0;
    vy *= 2.0;
    turn_control *= 2.0;

    MeltInput *melt_input = melt.get_input();

    melt_input->rpm = curr_rpm;
    melt_input->vx = vx;
    melt_input->vy = vy;
    melt_input->turn_control = turn_control;
    melt_input->target_rpm_control = target_rpm_control;
    // melt_input->led_shift_amount = led_shift_amount;
    melt_input->led_shift_amount = LED_SHIFT;

    // auto drive type used, manual drive type is irrelevant
    melt_input->manual_drive_type = DriveType::Melting;
    melt_input->auto_drive_type = true;

    // TODO: figure out sign
    Serial.printf("avg z: %.3f\n", get_average_z_accel());
    melt_input->is_upright = get_average_z_accel() >= 0;

    melt.general_update();
  } else {
    if (connected == ConnectionState::NetworkDisconnect) {
      Serial.println("no controller thingy");
      leds.leds_on_no_controller();
    } else if (connected == ConnectionState::ControllerDisconnect) {
      leds.leds_on_controller_stale();
    } else if (connected == ConnectionState::AccelDisconnect) {
      leds.leds_on_rgb(0, 255, 0);
    } else if (connected == ConnectionState::SpinSwitchOff) {
      leds.leds_on_switch_off();
    }
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
  telemetry_packet.radius = rpm_estimator.get_radius();
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
    leds.leds_on_array(output->led_red, output->led_green, output->led_blue);

    // if power low enough just break
    if (fabs(output->left_power) < 0.01 && fabs(output->right_power) < 0.01) {
      left_motor.send(0);
      right_motor.send(0);
    } else {
      left_motor.sendThrottle3D(output->left_power);
      right_motor.sendThrottle3D(output->right_power);
    }
  } else {
    // stop the motors unless strictly good for spinning
    left_motor.send(0);
    right_motor.send(0);
  }

  // perform every msec
  vTaskDelay(1);
}

void Robot::update_z_buffer(float z_accel) {
  z_accel_queue[z_accel_position] = z_accel;
  constexpr size_t num_elements =
      sizeof(z_accel_queue) / sizeof(z_accel_queue[0]);
  z_accel_position = (z_accel_position + 1) % num_elements;
}

float Robot::get_average_z_accel() {
  float avg = 0;
  constexpr size_t num_elements =
      sizeof(z_accel_queue) / sizeof(z_accel_queue[0]);

  for (size_t i = 0; i < num_elements; i++) {
    avg += z_accel_queue[i];
  }

  return avg / num_elements;
}
