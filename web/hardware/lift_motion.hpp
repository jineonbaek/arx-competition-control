#pragma once
#include "slow_lift_config.hpp"
#include <ruckig/ruckig.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

struct LiftState {
  double position=0.,velocity=0.,acceleration=0.,jerk=0.;
};

// Reference generation only: no motor, CAN, ROS, clock or SDK construction.
class LiftMotion {
 public:
  LiftMotion(double velocity=WEB_LIFT_DEFAULT_RAMP_RAD_S,
             double acceleration=WEB_LIFT_MAX_ACCELERATION,
             double jerk=WEB_LIFT_MAX_JERK):vmax_(velocity),amax_(acceleration),jmax_(jerk){
    if(!valid_lift_ramp_speed(velocity) || !std::isfinite(acceleration) ||
       acceleration<=0. || acceleration>WEB_LIFT_MAX_ACCELERATION ||
       !std::isfinite(jerk) || jerk<=0. || jerk>WEB_LIFT_MAX_JERK)
      throw std::invalid_argument("Invalid lift reference velocity/acceleration/jerk limits");
  }
  void request(double goal){
    if(!valid_lift_target(goal))throw std::invalid_argument("Lift target outside 0..18");
    if(initialized_ && goal==target_)return;  // Repeated web updates do not restart time.
    target_=goal;pending_=true;
  }
  void hold(double reference){
    if(!valid_lift_target(reference))throw std::invalid_argument("Invalid lift hold reference");
    state_={reference,0.,0.,0.};target_=reference;elapsed_=0.;
    initialized_=true;executing_=false;pending_=false;fault_.clear();
  }
  LiftState advance(double sdk_reference,double dt,bool calibrated){
    if(!valid_lift_target(sdk_reference))return fail(state_.position,"invalid_sdk_reference");
    if(!initialized_){state_={sdk_reference,0.,0.,0.};initialized_=true;}
    if(!calibrated){
      if(!ever_calibrated_){
        // Homing may change the reference. Retain the validated startup target
        // until the first calibrated tick, without advancing a trajectory.
        state_={sdk_reference,0.,0.,0.};executing_=false;
        return state_;
      }
      const bool interrupted=active();
      const std::string previous=fault_;
      hold(sdk_reference);
      fault_=interrupted?"calibration_lost":previous;
      return state_;
    }
    if(!ever_calibrated_){
      state_={sdk_reference,0.,0.,0.};ever_calibrated_=true;
      pending_=pending_ || target_!=sdk_reference;dt=0.;
    }
    if(std::abs(sdk_reference-state_.position)>1e-6)
      return fail(sdk_reference,"sdk_reference_mismatch");
    if(!std::isfinite(dt) || dt<0. || (dt>.05 && active()))
      return fail(sdk_reference,"reference_tick_gap");
    if(pending_){
      ruckig::InputParameter<1> input;
      input.current_position={state_.position};input.current_velocity={state_.velocity};
      input.current_acceleration={state_.acceleration};input.target_position={target_};
      input.target_velocity={0.};input.target_acceleration={0.};
      input.max_velocity={vmax_};input.max_acceleration={amax_};input.max_jerk={jmax_};
      ruckig::Trajectory<1> candidate;
      try{
        if(!solver_.validate_input<false>(input,true,true) || solver_.calculate(input,candidate)<0)
          return fail(sdk_reference,"trajectory_calculation_failed");
        const auto bounds=candidate.get_position_extrema()[0];
        if(!std::isfinite(candidate.get_duration()) || candidate.get_duration()>7000. ||
           !std::isfinite(bounds.min) || !std::isfinite(bounds.max) ||
           bounds.min< -1e-9 || bounds.max>18.+1e-9)
          return fail(sdk_reference,"trajectory_position_bounds");
      }catch(const std::exception&){return fail(sdk_reference,"trajectory_calculation_failed");}
      trajectory_=std::move(candidate);elapsed_=0.;pending_=false;
      executing_=trajectory_.get_duration()>0.;fault_.clear();++plans_;
    }
    if(executing_){
      elapsed_=std::min(trajectory_.get_duration(),elapsed_+dt);
      LiftState next;size_t section=0;
      trajectory_.at_time(elapsed_,next.position,next.velocity,next.acceleration,next.jerk,section);
      if(!std::isfinite(next.position) || !std::isfinite(next.velocity) ||
         !std::isfinite(next.acceleration) || !std::isfinite(next.jerk) ||
         next.position< -1e-9 || next.position>18.+1e-9 ||
         std::abs(next.velocity)>vmax_+1e-8 || std::abs(next.acceleration)>amax_+1e-8 ||
         std::abs(next.jerk)>jmax_+1e-8)
        return fail(sdk_reference,"invalid_trajectory_sample");
      next.position=std::clamp(next.position,0.,18.);state_=next;
      if(elapsed_>=trajectory_.get_duration()){
        state_={target_,0.,0.,0.};executing_=false;
      }
    }
    return state_;
  }
  double sdk_step_limit(double dt)const{
    // The SDK field is an increment per call, not a physical velocity. Cover
    // this tick's elapsed-time sample instead of clipping it at a fixed 1/400s.
    const double period=std::isfinite(dt) && dt>0. && dt<=.05 ? std::max(dt,1./400.) : 1./400.;
    return vmax_*period+1e-10;
  }
  const LiftState& state()const{return state_;}
  bool active()const{return executing_ || pending_;}
  double target()const{return target_;}
  double duration()const{return executing_?trajectory_.get_duration():0.;}
  const std::string& fault()const{return fault_;}
  uint64_t plan_count()const{return plans_;}
 private:
  LiftState fail(double reference,const char* reason){hold(reference);fault_=reason;return state_;}
  const double vmax_,amax_,jmax_;
  ruckig::Ruckig<1> solver_;
  ruckig::Trajectory<1> trajectory_;
  LiftState state_;
  double target_=0.,elapsed_=0.;
  bool initialized_=false,pending_=false,executing_=false,ever_calibrated_=false;
  uint64_t plans_=0;
  std::string fault_;
};
