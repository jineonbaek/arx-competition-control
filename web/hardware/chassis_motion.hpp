#pragma once
#include "slow_lift_config.hpp"
#include <algorithm>
#include <array>
#include <cmath>

// SDK input units per second. These are commissioning settings, not SI limits.
constexpr double WEB_LINEAR_ACCEL = 0.8;
constexpr double WEB_LINEAR_DECEL = 1.6;
constexpr double WEB_YAW_ACCEL = 1.6;
constexpr double WEB_YAW_DECEL = 0.8;
constexpr double WEB_REVERSE_HOLD_S = 0.2;
constexpr double WEB_BRAKE_HOLD_S = 0.8;

class ChassisMotion {
 public:
  using Velocity = std::array<double, 3>;

  void protect() {
    target_ = {};
    output_ = {};
    phase_ = Phase::Protect;
  }

  void stop(double now) {
    target_ = {};
    output_ = {};
    if (phase_ != Phase::Brake) {
      phase_ = Phase::Brake;
      deadline_ = now + WEB_BRAKE_HOLD_S;
    }
  }

  void request(const Velocity &target, int mode, double now) {
    if (!std::isfinite(now) ||
        !valid_chassis_target(target[0], target[1], target[2], mode)) {
      protect();
      return;
    }
    if (mode == 2 || target == Velocity{}) {
      stop(now);
      return;
    }
    // A new movement request cannot bypass an in-progress brake hold.
    if (phase_ == Phase::Brake) return;
    target_ = target;
    if (phase_ == Phase::Protect) phase_ = Phase::Drive;
  }

  void step(double now, double command_age, bool ready) {
    const double elapsed = now - last_step_;
    last_step_ = now;
    if (!ready || !std::isfinite(now) || !std::isfinite(elapsed) || elapsed < 0) {
      protect();
      return;
    }
    if (phase_ != Phase::Protect && phase_ != Phase::Brake && chassis_expired(command_age))
      stop(now);
    if (phase_ == Phase::Protect) return;
    if (phase_ == Phase::Brake) {
      if (now >= deadline_) protect();
      return;
    }
    if (phase_ == Phase::Hold) {
      if (now >= deadline_) phase_ = Phase::Drive;
      return;
    }
    if (phase_ == Phase::Drive) {
      for (std::size_t i = 0; i < output_.size(); ++i) {
        if (output_[i] * target_[i] < 0) {
          phase_ = Phase::Decelerate;
          break;
        }
      }
    }
    const Velocity goal = phase_ == Phase::Decelerate ? Velocity{} : target_;
    const double dt = std::min(elapsed, 0.02);
    const double dx = goal[0] - output_[0], dy = goal[1] - output_[1];
    const double distance = std::hypot(dx, dy);
    const double linear_rate = std::hypot(goal[0], goal[1]) < std::hypot(output_[0], output_[1])
                                   ? WEB_LINEAR_DECEL : WEB_LINEAR_ACCEL;
    if (distance <= linear_rate * dt) {
      output_[0] = goal[0];
      output_[1] = goal[1];
    } else {
      output_[0] += dx * linear_rate * dt / distance;
      output_[1] += dy * linear_rate * dt / distance;
    }
    const double yaw_rate = std::abs(goal[2]) < std::abs(output_[2]) ? WEB_YAW_DECEL : WEB_YAW_ACCEL;
    output_[2] += std::clamp(goal[2] - output_[2], -yaw_rate * dt, yaw_rate * dt);
    if (phase_ == Phase::Decelerate && output_ == Velocity{}) {
      phase_ = Phase::Hold;
      deadline_ = now + WEB_REVERSE_HOLD_S;
    }
  }

  const Velocity &velocity() const { return output_; }
  int mode() const { return phase_ == Phase::Protect ? 2 : 1; }

 private:
  enum class Phase { Protect, Drive, Decelerate, Hold, Brake };
  Velocity target_{}, output_{};
  Phase phase_ = Phase::Protect;
  double deadline_ = 0;
  double last_step_ = 0;
};
