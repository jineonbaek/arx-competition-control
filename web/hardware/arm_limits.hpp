#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>
struct ArmLimits {
  std::array<std::array<double,2>,6> degrees;
  std::array<double,2> gripper;
  double max_delta_deg;
  bool valid(const std::vector<double>& target, double grip,
             const std::vector<double>& actual, int mode) const {
    if (mode < 0 || mode > 5 || mode == 4 || target.size()!=6 || actual.size()!=7 || !std::isfinite(grip)) return false;
    for (int i=0;i<6;++i) {
      if (!std::isfinite(target[i]) || !std::isfinite(actual[i])) return false;
      if (mode==5) {
        double deg=target[i]*180.0/M_PI;
        if (deg<degrees[i][0]-1e-9 || deg>degrees[i][1]+1e-9) return false;
      }
    }
    return mode!=5 || valid_gripper(grip);
  }
  bool valid_gripper(double grip) const {
    return std::isfinite(grip) && grip>=gripper[0] && grip<=gripper[1];
  }
  std::vector<double> advance(const std::vector<double>& reference, const std::vector<double>& target,
                              const std::vector<double>& actual, double speed_deg_s, double dt) const {
    return advance(reference,target,actual,std::array<double,6>{speed_deg_s,speed_deg_s,speed_deg_s,speed_deg_s,speed_deg_s,speed_deg_s},dt);
  }
  std::vector<double> advance(const std::vector<double>& reference, const std::vector<double>& target,
                              const std::vector<double>& actual, const std::array<double,6>& speeds_deg_s, double dt) const {
    auto next=reference;
    double fraction=1;
    for(int i=0;i<6;++i) {
      double distance=std::abs(target[i]-reference[i]);
      if(distance>0) fraction=std::min(fraction,speeds_deg_s[i]*M_PI/180*dt/distance);
    }
    for(int i=0;i<6;++i) {
      next[i]+=fraction*(target[i]-reference[i]);
      if(std::abs(next[i]-actual[i])*180/M_PI>max_delta_deg+1e-9) return reference;
    }
    return next;
  }
};
inline double gripper_hold_target(const ArmLimits& limits,double measured,double commanded) {
  if(limits.valid_gripper(commanded))return commanded;
  return std::clamp(measured,limits.gripper[0],limits.gripper[1]);
}
// Only an already completed position reference is eligible. Active/fault stops
// keep their measured-position behavior; pending goals are never used here.
inline bool completed_hold_allowed(const std::string& reason,const std::string& status,int mode,bool trajectory_reference) {
  return reason=="manual_stop" && status=="reference_complete" && mode==5 && trajectory_reference;
}
inline std::vector<double> arm_stop_target(const ArmLimits& limits,
    const std::vector<double>& actual,const std::vector<double>& issued,bool completed,bool* retained=nullptr) {
  if(retained)*retained=false;
  if(actual.size()!=7 || !std::all_of(actual.begin(),actual.end(),[](double v){return std::isfinite(v);}))
    throw std::invalid_argument("Invalid stop feedback");
  if(completed && limits.valid(issued,limits.gripper[1],actual,5)) {
    bool bounded=true;
    for(size_t j=0;j<6;++j)
      bounded &= std::abs(issued[j]-actual[j])<=2.*M_PI/180.+1e-12;
    if(bounded){if(retained)*retained=true;return issued;}
  }
  return {actual.begin(),actual.begin()+6};
}
inline double startup_gripper_target(const ArmLimits& limits,double requested) {
  // NaN means unspecified: preserve the existing measured-position startup.
  if(std::isnan(requested))return requested;
  if(!limits.valid_gripper(requested))throw std::invalid_argument("Invalid startup gripper target");
  return requested;
}
inline int arm_feedback_index(unsigned id, unsigned dlc, unsigned first, bool local) {
  if (local) return -1;
  if (dlc>=6 && (id==1 || id==2 || id==4)) return id==4?2:id-1;
  unsigned motor=first&15;
  if (dlc==8 && (id==0 || id==motor) && motor>=5 && motor<=8) return motor-2;
  return -1;
}
