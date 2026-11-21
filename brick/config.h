#ifndef BRICK_CONFIG_H
#define BRICK_CONFIG_H

// This file has all the hard-coded settings for Potatomelt

#define NUM_LEDS 8

#define NEOPIXEL_PIN GPIO_NUM_1        // Top LED data
#define BOTTOM_NEOPIXEL_PIN GPIO_NUM_4 // Separate bottom LED data; rewire first
#define LEFT_MOTOR_PIN GPIO_NUM_2
#define RIGHT_MOTOR_PIN GPIO_NUM_3

#define I2C_SDA_PIN GPIO_NUM_5
#define I2C_SCL_PIN GPIO_NUM_6

// for wifi connection
#define HEARTBEAT_MS 50
#define TIMEOUT_MS 250

// Optional active-low snapshot button. Connect the chosen free GPIO to GND.
constexpr int DUMP_BUTTON_PIN = -1;

// Skip repeat ESC arming on an eligible restart with a valid retained marker.
// Cold starts, new firmware and lost/corrupt RTC state always arm normally.
constexpr bool RESUME_ARMING_ON_WARM_RESET = true;

// Boot defaults; a/t/n serial commands can select the mode until reboot.
constexpr bool RPM_HEADING_COMPENSATION = true;
// false: acceleration-only; translation uses plain dead reckoning.
constexpr bool RPM_HEADING_TRANSLATION_COMPENSATION = false;
// Calibrate the estimated correlation-window delay if required by real runs.
constexpr float RPM_HEADING_DELAY_SCALE = 1.f;
static_assert(RPM_HEADING_DELAY_SCALE >= 0 && RPM_HEADING_DELAY_SCALE <= 2,
              "Heading delay scale must be between 0 and 2");

// Translation uses a less aggressive correction/prediction profile. The actual
// measurement-clock mapping/delay scale is shared; filter delay is timestamped.
constexpr float RPM_HEADING_TRANSLATION_RESPONSE_MS = 200;
constexpr float RPM_HEADING_TRANSLATION_MAX_CORRECTION_RAD_S = 2;
constexpr float RPM_HEADING_TRANSLATION_PREDICTION_MS = 120;
static_assert(RPM_HEADING_TRANSLATION_RESPONSE_MS >= 100 &&
                  RPM_HEADING_TRANSLATION_MAX_CORRECTION_RAD_S > 0 &&
                  RPM_HEADING_TRANSLATION_MAX_CORRECTION_RAD_S <= 4 &&
                  RPM_HEADING_TRANSLATION_PREDICTION_MS > 0 &&
                  RPM_HEADING_TRANSLATION_PREDICTION_MS <= 120,
              "Invalid translation heading tuning");

// found from testing
#define LED_SHIFT -0.15

// look in pid.h for explanation of these values
#define KP 0.0f
#define KI 0.0f
// found from data fit
#define RPM_THROTTLE_FEEDFORWARD (1.f / (6784.08434167f * 1.20f))

// max +-0.05 throttle correction from integral
#define MAX_INTEGRAL 0.05f

#endif
