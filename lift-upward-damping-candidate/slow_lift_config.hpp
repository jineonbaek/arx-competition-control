#pragma once
#include <cmath>
// Original ramp() consumes radians PER CONTROL CYCLE at the fixed 400Hz loop.
template<class Config> inline void apply_slow_lift_config(Config& c) {
  c.lift_max_vel = 0.5 / 400.0;
  c.lift_calibrate_vel = 0.5;
}

inline bool valid_lift_target(double x) { return std::isfinite(x) && x >= 0.0 && x <= 18.3; }
