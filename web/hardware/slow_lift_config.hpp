#pragma once
#include <cmath>
constexpr double WEB_MAX_LINEAR = 0.80;
constexpr double WEB_MAX_YAW = 0.40;
constexpr double WEB_CHASSIS_TIMEOUT = 0.25;
template<class Config> inline void apply_slow_lift_config(Config& c) {
  c.lift_max_vel = 0.5 / 400.0;
  c.lift_calibrate_vel = 0.5;
}
inline bool valid_lift_target(double x) {
  return std::isfinite(x) && x >= 0.0 && x <= 18.0;
}
inline bool valid_chassis_target(double x, double y, double yaw, int mode,
                                 double linear_limit = WEB_MAX_LINEAR,
                                 double yaw_limit = WEB_MAX_YAW) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return false;
  if (mode == 2) return x == 0.0 && y == 0.0 && yaw == 0.0;
  return mode == 1 && std::hypot(x,y) <= linear_limit + 1e-9 &&
         std::abs(yaw) <= yaw_limit + 1e-9;
}
inline bool chassis_expired(double age) {
  return !std::isfinite(age) || age < 0.0 || age >= WEB_CHASSIS_TIMEOUT;
}
