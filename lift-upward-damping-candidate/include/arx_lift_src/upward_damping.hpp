#pragma once
#include <algorithm>
#include <cmath>
namespace arx_lift_experiment {
inline bool enabled = false;
inline bool motion_active = true;
inline bool active(bool on, bool calibrated, bool up, bool lift_type) { return on && calibrated && up && lift_type; }
// Experimental dissipative term in the existing SDK torque coordinate, not Nm.
inline double apply(double base, double velocity, bool on, double lower, double upper) {
  if (!on || !std::isfinite(base) || !std::isfinite(velocity) ||
      !std::isfinite(lower) || !std::isfinite(upper) || lower>upper ||
      base<lower || base>upper || std::abs(velocity)<=0.05) return base;
  const double delta=std::clamp(-0.2*velocity,-0.1,0.1);
  return std::clamp(base+delta,lower,upper);
}
}
