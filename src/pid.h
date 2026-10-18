#pragma once

class PID {
public:
  double kp;
  double ki;
  double kd;
  double p_on_e;
  double direct;

  PID(double kp, double ki, double kd, double p_on_e, double direct)
      : kp(kp), ki(ki), kd(kd), p_on_e(p_on_e), direct(direct) {}

	void update(double target, double measurement, double time){}
};
