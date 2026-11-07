#include "rpm_estimator.h"
#include <math.h>

RpmEstimator::RpmEstimator() {}

void RpmEstimator::update(float xg, float yg) {
  float mag = sqrt(xg * xg + yg * yg);

  // accel (cm/sec) = (omega (rad / sec))^2 * radius (cm)

  // omega (rot / min) =  omega (rad / sec) * (60 sec / 2pi rad)
  // ->
  // omega (rot / min) * (2pi rad / 60 sec) =  omega (rad / sec)
  // and
  // accel (cm / sec) = accel (G) * 9.81 (meters / sec) * (100 cm / 1 meters)
  // so

  // accel (G) * 9.81 (meters / sec) * (100 cm / 1 meters) = (omega (rot / min)
  // * (2pi rad / 60 sec))^2 * radius (cm)
  //
  // sqrt((accel (G) * 9.81*100) / (radius * (2pi rad / 60 sec)^2)) = omega
  //
  // C * sqrt(accel / radius) = omega
  //
  // C = sqrt(9.81 * 100 * (60 / 2pi)^2)
  // C = sqrt(9.81 * 100) * (30 / pi)
  const float C = (31.3209195267 * 30) / M_PI;

  float rpm = mag / radius;
  rpm = C * sqrt(rpm);

  update_filter(rpm);
  rpm_estimate = get_filter_estimate();
}

float RpmEstimator::get_estimated_rpm() { return rpm_estimate; }

void RpmEstimator::update_filter(float curr_rpm) {
  filter_estimate = curr_rpm * alpha + last_measurement * (1 - alpha);
  last_measurement = curr_rpm;
}
float RpmEstimator::get_filter_estimate() { return filter_estimate; }
