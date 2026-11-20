#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>
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
inline int arm_feedback_index(unsigned id, unsigned dlc, unsigned first, bool local) {
  if (local) return -1;
  if (dlc>=6 && (id==1 || id==2 || id==4)) return id==4?2:id-1;
  unsigned motor=first&15;
  if (dlc==8 && (id==0 || id==motor) && motor>=5 && motor<=8) return motor-2;
  return -1;
}
