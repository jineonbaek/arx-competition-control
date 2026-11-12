#pragma once
#include "arm_limits.hpp"
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_set>

struct TrajectoryPolicy {
  double speed_deg_s, acceleration_deg_s2, start_tolerance_deg, tracking_tolerance_deg;
  double max_tick_gap_s, lease_s, max_duration_s, start_speed_deg_s;
};

class ArmTrajectory {
  ArmLimits limits_;
  TrajectoryPolicy policy_;
  std::vector<std::vector<double>> points_;
  std::vector<double> times_;
  std::unordered_set<std::string> consumed_;
  std::string id_,state_="idle",reason_;
  double start_=0,last_tick_=0,last_lease_=0;
public:
  ArmTrajectory(const ArmLimits& limits,TrajectoryPolicy p):limits_(limits),policy_(p) {
    if(!std::isfinite(limits.max_delta_deg)||limits.max_delta_deg<=0)
      throw std::invalid_argument("Invalid tracking limit");
    for(const auto& range:limits.degrees)
      if(!std::isfinite(range[0])||!std::isfinite(range[1])||range[0]>=range[1])
        throw std::invalid_argument("Invalid joint limits");
    if(!std::isfinite(limits.gripper[0])||!std::isfinite(limits.gripper[1])||limits.gripper[0]>=limits.gripper[1])
      throw std::invalid_argument("Invalid gripper limits");
    for(double v:{p.speed_deg_s,p.acceleration_deg_s2,p.start_tolerance_deg,p.tracking_tolerance_deg,
                  p.max_tick_gap_s,p.lease_s,p.max_duration_s,p.start_speed_deg_s})
      if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid trajectory policy");
    if(p.speed_deg_s>5.7 || p.start_tolerance_deg>p.tracking_tolerance_deg ||
       p.tracking_tolerance_deg>limits.max_delta_deg || p.max_tick_gap_s>p.lease_s ||
       p.lease_s>p.max_duration_s || p.start_speed_deg_s>p.speed_deg_s)throw std::invalid_argument("Inconsistent trajectory policy");
  }
  bool active() const{return state_=="running";}
  const std::string& id() const{return id_;}
  const std::string& status() const{return state_;}
  const std::string& reason() const{return reason_;}
  void cancel(const std::string& reason){if(active()){state_="failed";reason_=reason;}}

  void start(const nlohmann::json& message,const std::vector<double>& actual,double now) {
    if(active())throw std::invalid_argument("Trajectory already active");
    auto id=message.at("request_id").get<std::string>();
    if(id.empty()||id.size()>64||id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_")!=std::string::npos)
      throw std::invalid_argument("Invalid request ID");
    if(consumed_.count(id)||consumed_.size()>=4096)throw std::invalid_argument("Trajectory ID consumed or capacity reached");
    if(message.at("version")!=1 || message.at("interpolation")!="segment_quintic_scalar")
      throw std::invalid_argument("Unsupported trajectory version/interpolation");
    auto times=message.at("time_from_start_s").get<std::vector<double>>();
    auto points=message.at("joint_waypoints_rad").get<std::vector<std::vector<double>>>();
    if(times.size()<2||times.size()>1024||points.size()!=times.size()||times.front()!=0||
       !std::isfinite(now)||actual.size()!=7||
       !std::all_of(actual.begin(),actual.end(),[](double v){return std::isfinite(v);}))
      throw std::invalid_argument("Invalid trajectory shape or actual state");
    for(size_t i=0;i<times.size();++i){
      if(!std::isfinite(times[i])||times[i]<0||times[i]>policy_.max_duration_s||
         !limits_.valid(points[i],limits_.gripper[1],actual,5))throw std::invalid_argument("Trajectory outside limits");
      if(i==0)continue;
      double dt=times[i]-times[i-1];
      if(dt<=0)throw std::invalid_argument("Non-increasing trajectory times");
    }
    points_=std::move(points);times_=std::move(times);id_=id;consumed_.insert(id);
    start_=last_tick_=last_lease_=now;state_="running";reason_.clear();
  }

  bool keepalive(const std::string& id,double now) {
    if(!active()||id!=id_)return false;
    if(!std::isfinite(now)||now<last_lease_||now<last_tick_){cancel("invalid_clock");return false;}
    if(now-last_lease_>policy_.lease_s){cancel("lease_expired");return false;}
    last_lease_=now;return true;
  }

  std::vector<double> tick(double now,const std::vector<double>& actual,bool healthy) {
    if(!active())return {};
    if(!std::isfinite(now)||now<last_tick_){cancel("invalid_clock");return {};}
    if(!healthy||actual.size()!=7||!std::all_of(actual.begin(),actual.end(),[](double v){return std::isfinite(v);})){
      cancel("feedback_invalid");return {};
    }
    if(now-last_tick_>policy_.max_tick_gap_s){cancel("timer_gap");return {};}
    if(now-last_lease_>policy_.lease_s){cancel("lease_expired");return {};}
    last_tick_=now;
    double elapsed=std::min(now-start_,times_.back());
    auto upper=std::upper_bound(times_.begin(),times_.end(),elapsed);
    size_t i=std::min(size_t(upper-times_.begin()-1),times_.size()-2);
    double u=(elapsed-times_[i])/(times_[i+1]-times_[i]);
    double s=u*u*u*(10+u*(-15+6*u));
    auto next=points_[i];
    for(size_t j=0;j<6;++j){
      next[j]+=s*(points_[i+1][j]-points_[i][j]);
      if(std::abs(next[j]-actual[j])*180/M_PI>policy_.tracking_tolerance_deg){cancel("tracking_error");return {};}
    }
    // End of the reference curve is not proof of measured arrival or a successful grasp.
    if(elapsed>=times_.back())state_="reference_complete";
    return next;
  }
};
