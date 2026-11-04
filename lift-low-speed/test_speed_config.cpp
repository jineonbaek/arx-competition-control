#include <arx_lift_src/lift_head_control_loop.h>
#include <cmath>
#include <limits>
#include <iostream>
#include "slow_lift_config.hpp"
int main() {
  arx::LiftHeadControlLoop::config c;
  c.lift_max_vel = 0.06;
  c.lift_calibrate_vel = 3.0;
  c.lift_kp = 2.0;
  c.gravity_compensation_torque = -2.0;
  c.lift_max_torque = 15.0;
  apply_slow_lift_config(c);
  if (c.lift_max_vel * 400.0 > 0.500001 || c.lift_max_vel <= 0) {
    std::cerr << "FAIL: at 400Hz the target ramp exceeds 0.5rad/s\n"; return 1;
  }
  if (std::abs(c.lift_max_vel * 400.0 - 0.5) > 1e-8) return 2;
  if (c.lift_calibrate_vel <= 0 || c.lift_calibrate_vel > 0.5) return 3;
  if (c.lift_kp != 2.0 || c.gravity_compensation_torque != -2.0 || c.lift_max_torque != 15.0) return 4;
  if (!valid_lift_target(0.0) || !valid_lift_target(18.3)) return 5;
  for (double x : {-0.01, 18.3001, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    if (valid_lift_target(x)) { std::cerr << "FAIL: unsafe target accepted\n"; return 6; }
  }
  std::cout << "PASS: target ramp0.5rad/s, homing0.5rad/s, gains and torque unchanged\n";
}
