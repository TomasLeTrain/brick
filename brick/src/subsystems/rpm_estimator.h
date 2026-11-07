#include "accelerometer.h"

class RpmEstimator {
public:
  RpmEstimator();
  void update(float xg, float yg);

  float get_estimated_rpm();

  void set_radius(float radius) { this->radius = radius; }
  void set_alpha(float alpha) { this->alpha = alpha; }

private:
  void update_filter(float curr_mag);
  float get_filter_estimate();

  float rpm_estimate;

  // for now filter is a simple exponential filter
  float last_measurement;
  float filter_estimate;

  // 1 - only current measurement, 0 - only last measurement
  float alpha;

  // radius in CM
  float radius;
};
