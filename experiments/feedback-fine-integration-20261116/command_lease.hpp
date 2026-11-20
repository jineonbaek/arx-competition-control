#pragma once
#include <cmath>
#include "slow_lift_config.hpp"
enum class CommandSource { None, Web, Gamepad };
class CommandLease {
 public:
  bool accept(CommandSource source, int mode, double now) {
    if (!std::isfinite(now) || (mode != 1 && mode != 2)) return false;
    // A web stop remains a global stop; a gamepad stop cannot interrupt web control.
    if (source == CommandSource::Web && mode == 2) {
      owner_ = source; deadline_ = now + WEB_CHASSIS_TIMEOUT; return true;
    }
    if (now < deadline_ && owner_ != source) return false;
    owner_ = source; deadline_ = now + WEB_CHASSIS_TIMEOUT; return true;
  }
 private:
  CommandSource owner_ = CommandSource::None;
  double deadline_ = 0;
};
