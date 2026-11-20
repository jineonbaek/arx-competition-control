#pragma once
#include <cmath>
#include <optional>
namespace arx_waypoints {
struct StartupCamera {double x,y,scale;};
inline std::optional<StartupCamera> startup_camera(double x,double y,double age,unsigned width) {
  // Same source-time bounds as navigation/pose_freshness.py. No extrapolation.
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(age)||age<-.1||age>.65||!width)return {};
  return StartupCamera{x,y,static_cast<double>(width)/10.};
}
}
