#pragma once
#include "slow_lift_config.hpp"
#include <algorithm>
#include <array>
#include <cmath>

constexpr double WEB_LINEAR_ACCEL = 1.6;
constexpr double WEB_LINEAR_DECEL = 1.6;
constexpr double WEB_YAW_ACCEL = 2.4;
constexpr double WEB_YAW_DECEL = 0.8;
constexpr double WEB_BRAKE_HOLD_S = 0.8;

class ChassisMotion {
 public:
  using Velocity = std::array<double, 3>;
  explicit ChassisMotion(double linear_limit=WEB_MAX_LINEAR, double yaw_limit=WEB_MAX_YAW)
      : linear_limit_(linear_limit), yaw_limit_(yaw_limit) {}

  void protect() {
    target_={}; output_={}; phase_=Phase::Protect; resumable_brake_=false;
  }
  void stop(double now) {
    resumable_brake_=false;
    target_={}; output_={};
    if (phase_!=Phase::Brake) {
      phase_=Phase::Brake; deadline_=now+WEB_BRAKE_HOLD_S;
    }
  }
  void pause(double now) {
    // Do not downgrade an existing explicit stop/watchdog brake.
    if (phase_==Phase::Brake) return;
    stop(now);resumable_brake_=true;
  }
  void request(const Velocity& target, int mode, double now, bool navigation_stage=false) {
    if (!std::isfinite(now) ||
        !valid_chassis_target(target[0],target[1],target[2],mode,linear_limit_,yaw_limit_)) {
      protect(); return;
    }
    if (mode==2 || target==Velocity{}) { stop(now); return; }
    // A newly accepted ordered target may resume after a temporary zero.
    // Never replay an old target: stop() erased it, and without this request
    // the existing brake deadline and stream timeout keep output at zero.
    if (phase_==Phase::Brake) {
      if (!resumable_brake_) return;
      phase_=Phase::Drive;resumable_brake_=false;
    }
    (void)navigation_stage; // Current SDK bounds are validated by Controller.
    target_=target;
    if (phase_==Phase::Protect) phase_=Phase::Drive;
  }
  void step(double now, double command_age, bool ready) {
    const double elapsed=now-last_step_; last_step_=now;
    if (!ready || !std::isfinite(now) || !std::isfinite(elapsed) || elapsed<0) {
      protect(); return;
    }
    if (phase_==Phase::Drive && chassis_expired(command_age)) stop(now);
    if (phase_==Phase::Protect) return;
    if (phase_==Phase::Brake) {
      if (now>=deadline_) protect();
      return;
    }
    Velocity goal=target_;
    const double dt=std::min(elapsed,.02);
    const double dx=goal[0]-output_[0],dy=goal[1]-output_[1];
    const double distance=std::hypot(dx,dy);
    const double rate=std::hypot(goal[0],goal[1])<std::hypot(output_[0],output_[1])
                        ? WEB_LINEAR_DECEL : WEB_LINEAR_ACCEL;
    if (distance<=rate*dt) {
      output_[0]=goal[0]; output_[1]=goal[1];
    } else {
      output_[0]+=dx*rate*dt/distance; output_[1]+=dy*rate*dt/distance;
    }
    const double yaw_rate=(output_[2]*goal[2]<0. || std::abs(goal[2])<std::abs(output_[2]))
                            ? WEB_YAW_DECEL : WEB_YAW_ACCEL;
    output_[2]+=std::clamp(goal[2]-output_[2],-yaw_rate*dt,yaw_rate*dt);
  }
  const Velocity& velocity() const { return output_; }
  int mode() const { return phase_==Phase::Protect ? 2 : 1; }

 private:
  enum class Phase { Protect, Drive, Brake };
  double linear_limit_,yaw_limit_;
  Velocity target_{},output_{};
  Phase phase_=Phase::Protect;
  bool resumable_brake_=false;
  double deadline_=0.,last_step_=0.;
};
