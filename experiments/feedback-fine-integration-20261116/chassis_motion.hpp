#pragma once
// OFFLINE CANDIDATE ONLY. Not installed in the running hardware controller.
#include "slow_lift_config.hpp"
#include <algorithm>
#include <array>
#include <cmath>

constexpr double WEB_LINEAR_ACCEL = 0.8;
constexpr double WEB_LINEAR_DECEL = 1.6;
constexpr double WEB_YAW_ACCEL = 1.6;
constexpr double WEB_YAW_DECEL = 0.8;
constexpr double WEB_REVERSE_HOLD_S = 0.2;
constexpr double WEB_BRAKE_HOLD_S = 0.8;

class ChassisMotion {
 public:
  using Velocity = std::array<double, 3>;
  explicit ChassisMotion(double linear_limit=WEB_MAX_LINEAR, double yaw_limit=WEB_MAX_YAW)
      : linear_limit_(linear_limit), yaw_limit_(yaw_limit) {}

  void protect() {
    target_={}; output_={}; phase_=Phase::Protect;
    axis_phase_.fill(AxisPhase::Drive);
    axis_deadline_.fill(0.);
  }
  void stop(double now) {
    target_={}; output_={};
    if (phase_!=Phase::Brake) {
      phase_=Phase::Brake; deadline_=now+WEB_BRAKE_HOLD_S;
    }
  }
  void request(const Velocity& target, int mode, double now) {
    if (!std::isfinite(now) ||
        !valid_chassis_target(target[0],target[1],target[2],mode,linear_limit_,yaw_limit_)) {
      protect(); return;
    }
    if (mode==2 || target==Velocity{}) { stop(now); return; }
    if (phase_==Phase::Brake) return;
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
    for (std::size_t i=0;i<3;++i) {
      if (axis_phase_[i]==AxisPhase::Hold && now>=axis_deadline_[i])
        axis_phase_[i]=AxisPhase::Drive;
      if (axis_phase_[i]==AxisPhase::Drive && output_[i]*target_[i]<0)
        axis_phase_[i]=AxisPhase::Decelerate;
      if (axis_phase_[i]!=AxisPhase::Drive) goal[i]=0.;
    }
    const double dt=std::min(elapsed,.02);
    const double dx=goal[0]-output_[0],dy=goal[1]-output_[1];
    const double distance=std::hypot(dx,dy);
    const double rate=std::hypot(goal[0],goal[1])<std::hypot(output_[0],output_[1])
                        ? WEB_LINEAR_DECEL : WEB_LINEAR_ACCEL;
    const bool reversing_x=axis_phase_[0]==AxisPhase::Decelerate;
    const bool reversing_y=axis_phase_[1]==AxisPhase::Decelerate;
    if (reversing_x || reversing_y) {
      // Reserve the joint XY slew budget for reaching zero in finite time.
      // Otherwise an ever-changing orthogonal target can make interpolation
      // approach zero forever without entering the reverse hold.
      const double rx=reversing_x ? -output_[0] : 0.;
      const double ry=reversing_y ? -output_[1] : 0.;
      const double reverse_distance=std::hypot(rx,ry);
      const double budget=rate*dt;
      const double used=std::min(reverse_distance,budget);
      if (reverse_distance<=budget) {
        if (reversing_x) output_[0]=0.;
        if (reversing_y) output_[1]=0.;
      } else {
        if (reversing_x) output_[0]+=rx*budget/reverse_distance;
        if (reversing_y) output_[1]+=ry*budget/reverse_distance;
      }
      const double remaining=std::sqrt(std::max(0.,budget*budget-used*used));
      const double nx=reversing_x ? 0. : dx;
      const double ny=reversing_y ? 0. : dy;
      const double other_distance=std::hypot(nx,ny);
      if (other_distance<=remaining) {
        if (!reversing_x) output_[0]=goal[0];
        if (!reversing_y) output_[1]=goal[1];
      } else {
        if (!reversing_x) output_[0]+=nx*remaining/other_distance;
        if (!reversing_y) output_[1]+=ny*remaining/other_distance;
      }
    } else if (distance<=rate*dt) {
      output_[0]=goal[0]; output_[1]=goal[1];
    } else {
      output_[0]+=dx*rate*dt/distance; output_[1]+=dy*rate*dt/distance;
    }
    const double yaw_rate=std::abs(goal[2])<std::abs(output_[2])
                            ? WEB_YAW_DECEL : WEB_YAW_ACCEL;
    output_[2]+=std::clamp(goal[2]-output_[2],-yaw_rate*dt,yaw_rate*dt);
    for (std::size_t i=0;i<3;++i) {
      if (axis_phase_[i]==AxisPhase::Decelerate && output_[i]==0.) {
        axis_phase_[i]=AxisPhase::Hold;
        axis_deadline_[i]=now+WEB_REVERSE_HOLD_S;
      }
    }
  }
  const Velocity& velocity() const { return output_; }
  int mode() const { return phase_==Phase::Protect ? 2 : 1; }

 private:
  enum class Phase { Protect, Drive, Brake };
  enum class AxisPhase { Drive, Decelerate, Hold };
  double linear_limit_,yaw_limit_;
  Velocity target_{},output_{};
  Phase phase_=Phase::Protect;
  std::array<AxisPhase,3> axis_phase_{};
  std::array<double,3> axis_deadline_{};
  double deadline_=0.,last_step_=0.;
};
