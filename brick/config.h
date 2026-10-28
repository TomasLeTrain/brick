// This file has all the hard-coded settings for Potatomelt

#define NEOPIXEL_PIN GPIO_NUM_3
// #define MOTOR_1_PIN GPIO_NUM_6
// #define MOTOR_2_PIN GPIO_NUM_7

//
#define NEOPIXEL_RMT RMT_CHANNEL_0
// #define MOTOR_1_RMT RMT_CHANNEL_1
// #define MOTOR_2_RMT RMT_CHANNEL_2

// // ------------ safety settings ----------------------
// #define CONTROL_UPDATE_TIMEOUT_MS 3000
//
// // ------------ Spin control settings ----------------
// #define ACCELEROMETER_HARDWARE_RADIUS_CM 5.13f
// // Ant-tato distance: 3.415f
// // Beetle-tato distance: 5.13f
//
// // ??
// #define LED_OFFSET_PERCENT 47
//
// // How quick steering while melting is (larger values = slower)
// #define LEFT_RIGHT_HEADING_CONTROL_DIVISOR 2.0f
//
// #define MIN_TRACKING_RPM 400
//
// // don't track heading if we are this slow (also puts upper limit on time spent
// // in melty loop for safety)
// #define MAX_TRACKING_ROTATION_INTERVAL_US                                      \
//   (1.0f / MIN_TRACKING_RPM) * 60 * 1000 * 1000
//
// #define MAX_TRACKING_RPM 3000;
//
// // ------------ control parameters -------------------
// #define CONTROL_TRANSLATE_DEADZONE 50
// #define CONTROL_SPIN_SPEED_DEADZONE 200
// #define CONTROL_THROTTLE_MINIMUM 500
//
// #define TANK_FORBACK_POWER_SCALE                                               \
//   0.02f // Scale the power waaaaay down on tank mode
// #define TANK_TURNING_POWER_SCALE                                               \
//   0.005f // because we're sitting on a pair of ungeared brushless motors
//
// // Proportional Gain - higher values give more sensitivity, lower values
// // give more stability
// #define PID_KP 1.0
// // Integral - damping on the rebound curves. Lower values = slower to
// // respond, but less bounces
// #define PID_KI 0.4
// // Derivative - useful to prevent overshoot of target value.
// #define PID_KD 0.0
//
// // ------------- controller button mappings ----------
// #define XBOX_DPAD_UP 0x01
// #define XBOX_DPAD_RIGHT 0x04
// #define XBOX_DPAD_DOWN 0x02
// #define XBOX_DPAD_LEFT 0x08
// #define XBOX_BUTTON_X 0x04
//
// // ------------ Pin and RMT Mappings -----------------
//
// #define NEOPIXEL_PIN GPIO_NUM_17
// #define MOTOR_1_PIN GPIO_NUM_6
// #define MOTOR_2_PIN GPIO_NUM_7
//
// #define NEOPIXEL_RMT RMT_CHANNEL_0
// #define MOTOR_1_RMT RMT_CHANNEL_1
// #define MOTOR_2_RMT RMT_CHANNEL_2
//
// #define I2C_SDA_PIN 8
// #define I2C_SCL_PIN 9
// #define BATTERY_ADC_PIN 10
//
// // ------------ Battery Configuration ---------------
//
// #define BATTERY_ALERT_ENABLED     // if enabled - heading LED will flicker when
//                                   // battery voltage is low
// #define BATTERY_CRIT_HALT_ENABLED // if enabled - robot will halt when battery
//                                   // voltage is critically low
// #define BATTERY_VOLTAGE_DIVIDER                                                \
//   8.24 // From the PCB - what's the voltage divider betweeen the battery + and
//        // the sense line?
// #define BATTERY_CELL_COUNT 4 // How many cells are in the battery?
// // Beetle-tato count: 4
// #define BATTERY_CELL_FULL_VOLTAGE                                              \
//   4.2 // What voltage is a fully-charged cell? Standard lipos are 4.2v, other
//       // chemistries will vary
// #define BATTERY_CELL_EMPTY_VOLTAGE                                             \
//   3.2 // And on the other hand, what voltage is an empty cell? We're going to
//       // cut off at 3.2v/cell
// #define LOW_BAT_REPEAT_READS_BEFORE_ALARM                                      \
//   20 // Requires this many ADC reads below threshold before halting the robot
