#ifndef BRICK_CONFIG_H
#define BRICK_CONFIG_H

// This file has all the hard-coded settings for Potatomelt

#define NUM_LEDS 8

// #define NEOPIXEL_PIN GPIO_NUM_2
#define NEOPIXEL_PIN GPIO_NUM_1
#define LEFT_MOTOR_PIN GPIO_NUM_2
#define RIGHT_MOTOR_PIN GPIO_NUM_3

#define I2C_SDA_PIN GPIO_NUM_5
#define I2C_SCL_PIN GPIO_NUM_6

// for wifi connection
#define HEARTBEAT_MS 50
#define TIMEOUT_MS 250

// // ------------ Spin control settings ----------------
// #define ACCELEROMETER_HARDWARE_RADIUS_CM 4.668f
#define ACCELEROMETER_HARDWARE_RADIUS_CM 4.846f

// Compare both estimates on Serial before opting into RSSI for drive control.
constexpr bool USE_RSSI_RPM_FOR_CONTROL = false;

// Optional active-low snapshot button. Connect the chosen free GPIO to GND.
constexpr int DUMP_BUTTON_PIN = -1;

// found from testing
#define LED_SHIFT -0.15

// look in pid.h for explanation of these values
#define KP 0.0f
#define KI 0.0f
#define RPM_THROTTLE_FEEDFORWARD (1.f / 6784.08434167f)
// using throttle with bidirectional while issues get figured out, feedforward needs to get scaled accordingly
// #define RPM_THROTTLE_FEEDFORWARD (2.f / 6784.08434167f)

// max +-0.05 throttle correction from integral
#define MAX_INTEGRAL 0.05f

#endif
