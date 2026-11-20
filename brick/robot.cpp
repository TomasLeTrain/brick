#include "robot.h"
#include "config.h"
#include "src/subsystems/led.h"
#include "src/subsystems/melt.h"
#include <cstring>
#include <esp_system.h>

Robot::Robot() {}

void Robot::begin() {
  const esp_reset_reason_t reset = esp_reset_reason();
  const char *reason = "other";
  switch (reset) {
  case ESP_RST_POWERON:
    reason = "power-on";
    break;
  case ESP_RST_SW:
    reason = "software";
    break;
  case ESP_RST_PANIC:
    reason = "panic";
    break;
  case ESP_RST_INT_WDT:
    reason = "interrupt-watchdog";
    break;
  case ESP_RST_TASK_WDT:
    reason = "task-watchdog";
    break;
  case ESP_RST_WDT:
    reason = "watchdog";
    break;
  case ESP_RST_BROWNOUT:
    reason = "brownout";
    break;
  default:
    break;
  }
  Serial.printf("# boot reset_reason=%d (%s)\n", int(reset), reason);
  // TODO: check for any begin errors and report with leds
  // leds.set_brightness(30);
  leds.set_brightness(30);
  bottom_leds.set_brightness(30);

  left_motor.begin(LEFT_MOTOR_PIN, DSHOT600, false, 14);
  right_motor.begin(RIGHT_MOTOR_PIN, DSHOT600, false, 14);

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
  ThrottlePacket *throttle_packet = comms.get_throttle_packet();

  if (!comms.connected()) {
    connected = ConnectionState::NetworkDisconnect;
  } else if (!throttle_packet->connected) {
    connected = ConnectionState::ControllerDisconnect;
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
                     config.melt_max_throttle_magnitude)
          : 0;
  const float steady_rpm = spin_throttle / RPM_THROTTLE_FEEDFORWARD;
  const float command_x = (throttle_packet->right_ud / 1000.f - .5f) * 2;
  const float command_y = (throttle_packet->right_lr / 1000.f - .5f) * 2;
  const bool translating =
      connected == ConnectionState::Ready &&
      std::hypot(command_x, command_y) / M_SQRT2 >= config.melt_deadzone;
  comms.update_rpm(spin_throttle, steady_rpm, translating);
  const auto &rssi_estimator = comms.get_rssi_estimator();
  const auto &diagnostic = rssi_estimator.diagnostics();
  const float curr_rpm = rssi_estimator.rpm();
  const auto &heading = comms.get_heading_tracker().state();
  melt.set_phase_reference(heading.timestamp_us, heading.heading_rad,
                           heading.rpm);

  RunDump::Record sample{};
  sample.timestamp_us = heading.timestamp_us;
  sample.received_us = micros();
  sample.heading_rad = heading.heading_rad;
  sample.heading_error_rad = heading.error_rad;
  sample.rpm_age_ms = heading.measurement_age_ms;
  sample.target_rpm = steady_rpm;
  sample.rssi_rpm = curr_rpm;
  sample.control_rpm = heading.rpm;
  sample.rssi_hz = rssi_estimator.sample_rate_hz();
  sample.throttle = spin_throttle;
  sample.fast_rpm = diagnostic.fast_rpm;
  sample.long_rpm = diagnostic.long_rpm;
  sample.correlation = diagnostic.correlation;
  sample.analysis_timestamp_us = diagnostic.timestamp_us;
  sample.comparison_samples = diagnostic.comparison_samples;
  sample.min_lag = diagnostic.min_lag;
  sample.max_lag = diagnostic.max_lag;
  sample.tracking_mode =
      diagnostic.mode | (static_cast<uint8_t>(diagnostic.status) << 2);
  sample.hint_used =
      uint8_t(diagnostic.hint_used) | (uint8_t(heading.status) << 1);
  sample.estimator_us = comms.estimator_time_us();
  sample.rx_dropped = comms.dropped_rx();
  sample.rssi_dropped = comms.dropped_rssi();
  sample.connected = 0x80 | (connected == ConnectionState::Ready) |
                     (uint8_t(connected.load()) << 1) |
                     (throttle_packet->connected ? 0x10 : 0) |
                     (throttle_packet->swa ? 0x20 : 0) |
                     (throttle_packet->swb ? 0x40 : 0);
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

    melt_input->is_upright = !throttle_packet->inverted;
    melt.general_update();
  }

  // Diagnostics must not block the control loop when USB isn't being drained.
  static ConnectionState previous_state = ConnectionState::NetworkDisconnect;
  const ConnectionState state = connected.load();
  if (state != previous_state) {
    previous_state = state;
    const char *messages[] = {"# network disconnected\n",
                              "# controller disconnected\n",
                              "# spin disabled\n", "", "# ready\n"};
    const char *message = messages[int(state)];
    const size_t length = std::strlen(message);
    if (!comms.get_recorder().printing() &&
        Serial.availableForWrite() >= int(length))
      Serial.write(reinterpret_cast<const uint8_t *>(message), length);
  }

  // Versioned telemetry contains only live control/RPM/link diagnostics.
  TelemetryPacket telemetry_packet{};
  telemetry_packet.micros_timestamp = micros();
  telemetry_packet.rpm = curr_rpm;
  telemetry_packet.target_rpm = steady_rpm;
  telemetry_packet.throttle = spin_throttle;
  telemetry_packet.rssi_hz = rssi_estimator.sample_rate_hz();
  telemetry_packet.correlation = diagnostic.correlation;
  telemetry_packet.heading_rad = heading.heading_rad;
  telemetry_packet.heading_error_rad = heading.error_rad;
  telemetry_packet.rpm_age_ms = heading.measurement_age_ms;
  telemetry_packet.rx_dropped = comms.dropped_rx();
  telemetry_packet.rssi_dropped = comms.dropped_rssi();
  telemetry_packet.tx_failures = comms.tx_failures();
  telemetry_packet.connection_state = connected.load();
  telemetry_packet.rpm_status = uint8_t(diagnostic.status);
  telemetry_packet.heading_status = uint8_t(heading.status);
  telemetry_packet.inverted = throttle_packet->inverted;
  telemetry_packet.companion_active = throttle_packet->companion_active;
  telemetry_packet.controller_connected = throttle_packet->connected;
  telemetry_packet.spin_enabled = throttle_packet->swa;
  telemetry_packet.rssi_dbm = comms.last_rssi_dbm();
  telemetry_packet.control_age_ms = comms.control_age_ms();
  telemetry_packet.control_sequence = throttle_packet->sequence;

  comms.send(&telemetry_packet);
  // A neutral controller is idle even with SWA enabled. Check both current
  // drive intent (including tank movement) and the motor task's zero ACK.
  const bool no_drive_command =
      connected != ConnectionState::Ready || !melt.has_motor_command();
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
           !motor_max_gap_us.compare_exchange_weak(maximum, gap)) {
    }
  }
  previous_us = now_us;
  const uint32_t storage_token = storage_gate.token();
  if (!arming && StorageGate::requested(storage_token)) {
    if (!storage_gate.acknowledged(storage_token)) {
      left_motor.send(0);
      right_motor.send(0);
      leds.set_brightness(30);
      bottom_leds.set_brightness(30);
      motor_active.store(false);
      // Finish all queued peripheral transactions before flash can disable
      // caches/interrupts. No RMT calls follow the ACK until parking is
      // revoked.
      if (left_motor.wait_tx_idle(20) && right_motor.wait_tx_idle(20) &&
          leds.wait_idle(20) && bottom_leds.wait_idle(20))
        storage_gate.acknowledge(storage_token);
    }
    vTaskDelay(1);
    return;
  }
  if (arming) {
    // One startup message is sufficient; never print at the 1 kHz motor rate.
    static bool announced = false;
    if (!announced && Serial.availableForWrite() >= 9) {
      const uint8_t message[] = "# arming\n";
      Serial.write(message, sizeof(message) - 1);
      announced = true;
    }
    motor_active.store(true);
    left_motor.send(command);
    right_motor.send(command);
  } else if (connected == ConnectionState::Ready &&
             comms.motor_control_fresh()) {
    melt.fast_update();
    const MeltOutput *output = melt.get_output();
    leds.leds_on_array(output->led_red, output->led_green, output->led_blue);
    // The bottom face uses the opposite palette; manual flip swaps the faces.
    bottom_leds.leds_on_array(output->led_red, output->led_green,
                              output->led_blue, true);

    // if power low enough just break
    if (fabs(output->left_power) < 0.0001 &&
        fabs(output->right_power) < 0.0001) {
      left_motor.send(0);
      right_motor.send(0);
      leds.set_brightness(30);
      bottom_leds.set_brightness(30);
      motor_active.store(false);
    } else {
      motor_active.store(true);
      left_motor.sendThrottle3D(-output->left_power * 100.f);
      right_motor.sendThrottle3D(output->right_power * 100.f);
      leds.set_brightness(150);
      bottom_leds.set_brightness(150);
    }
  } else {
    // stop the motors unless strictly good for spinning
    left_motor.send(0);
    right_motor.send(0);
    leds.set_brightness(30);
    bottom_leds.set_brightness(30);
    motor_active.store(false);
    // Keep every runtime LED/RMT call on this task, including switch-off and
    // disconnect indications. The general task must not submit after park ACK.
    static uint32_t last_status_led_us = 0;
    if (now_us - last_status_led_us >= 10000) {
      last_status_led_us = now_us;
      if (connected == ConnectionState::NetworkDisconnect)
        leds.leds_on_no_controller();
      else if (connected == ConnectionState::ControllerDisconnect ||
               connected == ConnectionState::Ready)
        leds.leds_on_controller_stale();
      else if (connected == ConnectionState::SpinSwitchOff)
        leds.leds_on_switch_off();
      if (connected == ConnectionState::NetworkDisconnect)
        bottom_leds.leds_on_no_controller();
      else if (connected == ConnectionState::ControllerDisconnect ||
               connected == ConnectionState::Ready)
        bottom_leds.leds_on_controller_stale();
      else if (connected == ConnectionState::SpinSwitchOff)
        bottom_leds.leds_on_switch_off();
    }
  }

  // perform every msec
  vTaskDelay(1);
}
