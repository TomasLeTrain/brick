#include "robot.h"
#include "config.h"
#include "src/subsystems/led.h"
#include "src/subsystems/melt.h"
#include <Wire.h>
#include <esp_system.h>

Robot::Robot() {}

void Robot::begin() {
  const esp_reset_reason_t reset = esp_reset_reason();
  const char *reason = "other";
  switch (reset) {
  case ESP_RST_POWERON: reason = "power-on"; break;
  case ESP_RST_SW: reason = "software"; break;
  case ESP_RST_PANIC: reason = "panic"; break;
  case ESP_RST_INT_WDT: reason = "interrupt-watchdog"; break;
  case ESP_RST_TASK_WDT: reason = "task-watchdog"; break;
  case ESP_RST_WDT: reason = "watchdog"; break;
  case ESP_RST_BROWNOUT: reason = "brownout"; break;
  default: break;
  }
  Serial.printf("# boot reset_reason=%d (%s)\n", int(reset), reason);
  // TODO: check for any begin errors and report with leds
  leds.set_brightness(30);

  left_motor.begin(LEFT_MOTOR_PIN, DSHOT600, false, 14);
  right_motor.begin(RIGHT_MOTOR_PIN, DSHOT600, false, 14);

  // Serial.println("init accel");

  // set up I2C
  // Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // Wire.setClock(400'000);
  // // 10 msec timeout should be more than enough
  // Wire.setTimeout(10);
  // Serial.println("init i2c");

  // accel.init(0x18);

  // small wait needed before interacting with accelerometer
  // delay(20);

  // // try and sample once to see if there is a connection error or not
  // accel.sample_offset();

  // // only calibrate if accel is properly connected
  // if (!accel.has_connection_error()) {
  //   // sample offset
  //   // TODO: move to other place/when should this happen?
  //   for (int i = 0; i < 60; i++) {
  //     accel.sample_offset();
  //     delay(1);
  //   }

  //   Serial.println("did accel");
  // } else {
  //   Serial.println("failed accel init!");
  // }

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
  // for (int i = 0; i < sizeof(z_accel_queue) / sizeof(z_accel_queue[0]); i++) {
  //   z_accel_queue[i] = 0.0;
  // }
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
  // accel.update_reading();
  int32_t accel_measurement_micros = micros();
  // accel_disconnected = accel.has_connection_error();
  accel_disconnected = true;

  float x = 0;
  float y = 0;
  float z = 0;
  uint16_t raw_x = 0;
  uint16_t raw_y = 0;
  uint16_t raw_z = 0;

  // float x = accel.get_x_accel(false);
  // float y = accel.get_y_accel(false);
  // float z = accel.get_z_accel(false);
  // uint16_t raw_x = accel.get_raw_x_accel();
  // uint16_t raw_y = accel.get_raw_y_accel();
  // uint16_t raw_z = accel.get_raw_z_accel();

  // update z accel buffer
  update_z_buffer(z);

  ThrottlePacket *throttle_packet = comms.get_throttle_packet();

  if (!comms.connected()) {
    if (!comms.get_recorder().printing())
      Serial.println("network dc");
    connected = ConnectionState::NetworkDisconnect;
  } else if (!throttle_packet->connected) {
    connected = ConnectionState::ControllerDisconnect;
  // } else if (accel_disconnected) {
  //   connected = ConnectionState::AccelDisconnect;
  } else if (!throttle_packet->swa) {
    connected = ConnectionState::SpinSwitchOff;
  } else {
    connected = ConnectionState::Ready;
  }

  const auto &config = *melt.get_config();
  const float requested_speed = throttle_packet->left_ud / 1000.0f;
  const float target_rpm = requested_speed * config.max_target_rpm;
  const float spin_throttle =
      connected == ConnectionState::Ready &&
      requested_speed >= config.target_rpm_control_deadzone
          ? std::min(target_rpm * RPM_THROTTLE_FEEDFORWARD,
                     config.melt_max_throttle_magnitude) : 0;
  const float steady_rpm = spin_throttle / RPM_THROTTLE_FEEDFORWARD;
  comms.update_rpm(spin_throttle, steady_rpm);
  const auto &rssi_estimator = comms.get_rssi_estimator();
  const auto &diagnostic = rssi_estimator.diagnostics();
  const float curr_rpm = rssi_estimator.rpm();

  RunDump::Record sample{};
  sample.timestamp_us = accel_measurement_micros;
  sample.received_us = micros();
  sample.ax_g = x;
  sample.ay_g = y;
  sample.az_g = z;
  sample.target_rpm = steady_rpm;
  sample.rssi_rpm = curr_rpm;
  sample.control_rpm = curr_rpm;
  sample.rssi_hz = rssi_estimator.sample_rate_hz();
  sample.throttle = spin_throttle;
  sample.fast_rpm = diagnostic.fast_rpm;
  sample.long_rpm = diagnostic.long_rpm;
  sample.correlation = diagnostic.correlation;
  sample.analysis_timestamp_us = diagnostic.timestamp_us;
  sample.comparison_samples = diagnostic.comparison_samples;
  sample.min_lag = diagnostic.min_lag;
  sample.max_lag = diagnostic.max_lag;
  sample.tracking_mode = diagnostic.mode |
      (static_cast<uint8_t>(diagnostic.status) << 2);
  sample.hint_used = diagnostic.hint_used;
  sample.estimator_us = comms.estimator_time_us();
  sample.rx_dropped = comms.dropped_rx();
  sample.rssi_dropped = comms.dropped_rssi();
  sample.connected = 0x80 | (connected == ConnectionState::Ready) |
      (uint8_t(connected.load()) << 1) | (throttle_packet->connected ? 0x10 : 0) |
      (throttle_packet->swa ? 0x20 : 0) | (throttle_packet->swb ? 0x40 : 0);
  sample.motor_gap_us = motor_max_gap_us.exchange(0);
  comms.get_recorder().add_state(sample);

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

    // Serial.printf("avg z: %.3f\n", get_average_z_accel());
    // melt_input->is_upright = get_average_z_accel() >= 0;
    // bool curr_upright = get_average_z_accel() >= -1.0;
    bool curr_upright = true;

    if (curr_upright != last_upright) {
      different_upright_time_msec += 10;
      if (different_upright_time_msec > upright_time_threshold) {
        last_upright = curr_upright;
        different_upright_time_msec = 0;
      }
    } else {
      different_upright_time_msec = 0;
    }

    melt_input->is_upright = last_upright;
    // melt_input->is_upright = true;

    melt.general_update();
  } else {
    if (connected == ConnectionState::NetworkDisconnect) {
      if (!comms.get_recorder().printing())
        Serial.println("no controller thingy");
    }
  }

  static uint32_t lastMessage = 0;
  int printing_time = 20;

  if (comms.connected()) {
    if (!comms.get_recorder().printing() &&
        uint32_t(millis() - lastMessage) >= printing_time) {
      // Serial.printf("l_lr=%d,l_ud=%d,r_lr=%d,r_ud=%d,kl=%d,kr=%d,c=%d\n",
      //               throttle_packet->left_lr, throttle_packet->left_ud,
      //               throttle_packet->right_lr, throttle_packet->right_ud,
      //               throttle_packet->knob_left, throttle_packet->knob_right,
      //               throttle_packet->connected);
      Serial.printf("x:%.3f,y:%.3f,z:%.3f,rpm:%.1f,fast_rpm:%.1f,"
                    "long_rpm:%.1f,throttle:%.3f,rssi_hz:%.0f\n",
                    x, y, z, curr_rpm, diagnostic.fast_rpm,
                    diagnostic.long_rpm, spin_throttle,
                    rssi_estimator.sample_rate_hz());
      lastMessage = millis();
    }
  } else {
    if (!comms.get_recorder().printing())
      Serial.println("disconnected");
  }

  // now send telemetry
  TelemetryPacket telemetry_packet;
  telemetry_packet.a_x = raw_x;
  telemetry_packet.a_y = raw_y;
  telemetry_packet.a_z = raw_z;
  telemetry_packet.rpm = curr_rpm;
  telemetry_packet.radius = 0; // Reserved legacy telemetry slot.
  telemetry_packet.micros_timestamp = accel_measurement_micros;

  comms.send(&telemetry_packet);
  // A neutral controller is idle even with SWA enabled. Check both current
  // drive intent (including tank movement) and the motor task's zero ACK.
  const bool no_drive_command = connected != ConnectionState::Ready ||
                                !melt.has_motor_command();
  storage_gate.request(!arming && no_drive_command &&
                       comms.get_recorder().filesystem_pending());
  comms.update_recorder(!arming && no_drive_command && !motor_active.load() &&
                        storage_gate.parked());

  // perform every 10 msec
  vTaskDelay(10);
}

void Robot::motor_update() {
  static uint32_t previous_us = 0;
  const uint32_t now_us = micros();
  if (previous_us) {
    const uint32_t gap = now_us - previous_us;
    uint32_t maximum = motor_max_gap_us.load();
    while (gap > maximum &&
           !motor_max_gap_us.compare_exchange_weak(maximum, gap)) {}
  }
  previous_us = now_us;
  const uint32_t storage_token = storage_gate.token();
  if (!arming && StorageGate::requested(storage_token)) {
    if (!storage_gate.acknowledged(storage_token)) {
      left_motor.send(0);
      right_motor.send(0);
      motor_active.store(false);
      // Finish all queued peripheral transactions before flash can disable
      // caches/interrupts. No RMT calls follow the ACK until parking is revoked.
      if (left_motor.wait_tx_idle(20) && right_motor.wait_tx_idle(20) &&
          leds.wait_idle(20))
        storage_gate.acknowledge(storage_token);
    }
    vTaskDelay(1);
    return;
  }
  if (arming) {
    if (!comms.get_recorder().printing())
      Serial.println("arming");
    motor_active.store(true);
    left_motor.send(command);
    right_motor.send(command);
  } else if (connected == ConnectionState::Ready) {
    melt.fast_update();
    const MeltOutput *output = melt.get_output();
    leds.leds_on_array(output->led_red, output->led_green, output->led_blue);

    // if power low enough just break
    if (fabs(output->left_power) < 0.0001 &&
        fabs(output->right_power) < 0.0001) {
      left_motor.send(0);
      right_motor.send(0);
      motor_active.store(false);
    } else {
      motor_active.store(true);
      // left_motor.sendThrottle(output->left_power);
      // right_motor.sendThrottle(output->right_power);
      left_motor.sendThrottle3D(-output->left_power * 100.f);
      right_motor.sendThrottle3D(-output->right_power * 100.f);
    }
    //
    // left_motor.sendThrottle3D(output->left_power);
    // right_motor.sendThrottle3D(output->right_power);

    // left_motor.sendThrottle(output->left_power);
    // right_motor.sendThrottle(output->right_power);
  } else {
    // stop the motors unless strictly good for spinning
    left_motor.send(0);
    right_motor.send(0);
    motor_active.store(false);
    // Keep every runtime LED/RMT call on this task, including switch-off and
    // disconnect indications. The general task must not submit after park ACK.
    static uint32_t last_status_led_us = 0;
    if (now_us - last_status_led_us >= 10000) {
      last_status_led_us = now_us;
      if (connected == ConnectionState::NetworkDisconnect)
        leds.leds_on_no_controller();
      else if (connected == ConnectionState::ControllerDisconnect)
        leds.leds_on_controller_stale();
      else if (connected == ConnectionState::AccelDisconnect)
        leds.leds_on_rgb(0, 255, 0);
      else if (connected == ConnectionState::SpinSwitchOff)
        leds.leds_on_switch_off();
    }
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
