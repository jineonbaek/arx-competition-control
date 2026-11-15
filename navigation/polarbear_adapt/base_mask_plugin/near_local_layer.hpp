#pragma once
#include "geometry.hpp"
#include "near_local_grid.hpp"
#include <chrono>
#include <mutex>
#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace arx_base_mask {
class NearLocal {
 using Message=nav2_msgs::msg::Costmap;
 using Steady=std::chrono::steady_clock;
 struct Authority {Pose map,odom;double received;};
 std::mutex mutex_;
 std::optional<Authority> authority_,frozen_authority_;
 Message::ConstSharedPtr local_,frozen_local_;
 rclcpp::Subscription<Message>::SharedPtr local_sub_;
 rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
 double radius_=0.,max_age_=.5,max_pose_age_=.65,max_gap_=.35;
 double local_received_=0.,last_local_=-1.,last_pose_=-1.,last_now_=-1.;
 Window window_;
 bool ready_=false,prepared_=false;
 std::vector<std::pair<unsigned,unsigned char>> override_;
 static double wall(){return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();}
 static bool fresh(double stamp,double now,double age){return std::isfinite(stamp)&&now-stamp>=-.1&&now-stamp<=age;}
 static bool pose_valid(const geometry_msgs::msg::Pose& p){
  const auto&q=p.orientation;const auto&t=p.position;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return std::isfinite(t.x)&&std::isfinite(t.y)&&std::isfinite(t.z)&&std::isfinite(norm)&&std::abs(norm-1.)<.02;
 }
 static near_local::Grid view(const Message& m){
  const auto&v=m.metadata;const auto&p=v.origin;
  return {v.size_x,v.size_y,v.resolution,p.position.x,p.position.y,tf2::getYaw(p.orientation),&m.data};
 }
 public:
 bool enabled()const{return radius_>0.;}
 bool ready()const{return !enabled()||ready_;}
 void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node,const std::string&name,const std::string&frame){
  auto parameter=[&](const std::string&key,double value){const auto p=name+"."+key;if(!node->has_parameter(p))node->declare_parameter(p,value);return node->get_parameter(p).as_double();};
  radius_=parameter("near_local_radius",0.);
  if(!std::isfinite(radius_)||radius_<0.||radius_>3.)throw std::runtime_error("Invalid Local-priority radius");
  if(!enabled())return;
  if(frame!="map")throw std::runtime_error("Local-priority replacement is Global-only");
  max_age_=parameter("near_local_max_age",.5);max_pose_age_=parameter("near_local_pose_max_age",.65);max_gap_=parameter("near_local_max_gap",.35);
  for(double x:{max_age_,max_pose_age_,max_gap_})if(!std::isfinite(x)||x<=0.||x>1.)throw std::runtime_error("Invalid near-local freshness limit");
  local_sub_=node->create_subscription<Message>("/local_costmap/costmap_raw",rclcpp::QoS(1).transient_local(),[this](Message::ConstSharedPtr m){
   std::lock_guard<std::mutex> lock(mutex_);
   const auto&q=m->metadata.origin.orientation;
   const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
   if(m->header.frame_id!="odom"||!std::isfinite(norm)||std::abs(norm-1.)>=.02||std::abs(q.x)>1e-6||std::abs(q.y)>1e-6||!near_local::valid(view(*m))){local_.reset();return;}
   const double stamp=rclcpp::Time(m->header.stamp).seconds();
   if(stamp<=last_local_)return;
   last_local_=stamp;local_=m;local_received_=wall();
  });
  pose_sub_=node->create_subscription<geometry_msgs::msg::PoseArray>("/pb/chassis_pose",rclcpp::QoS(1).transient_local(),[this](geometry_msgs::msg::PoseArray::ConstSharedPtr m){
   std::lock_guard<std::mutex> lock(mutex_);
   if(m->header.frame_id!="map"||m->poses.size()!=2||!pose_valid(m->poses[0])||!pose_valid(m->poses[1])){authority_.reset();return;}
   const double stamp=rclcpp::Time(m->header.stamp).seconds();if(stamp<=last_pose_)return;
   auto pose=[&](const auto&p){return Pose{p.position.x,p.position.y,tf2::getYaw(p.orientation),stamp};};
   last_pose_=stamp;authority_=Authority{pose(m->poses[0]),pose(m->poses[1]),wall()};
  });
  RCLCPP_INFO(node->get_logger(),"%s uses Local raw costs within %.3fm; unavailable Local/pose blocks near-field planning",name.c_str(),radius_);
 }
 void update(double x,double y,double now){
  if(!enabled())return;
  std::lock_guard<std::mutex> lock(mutex_);
  if(last_now_>=0.&&now<last_now_){authority_.reset();local_.reset();}
  last_now_=now;
  if(authority_&&(!fresh(authority_->map.stamp,now,max_pose_age_)||wall()-authority_->received>max_gap_))authority_.reset();
  if(local_&&(!fresh(rclcpp::Time(local_->header.stamp).seconds(),now,max_age_)||wall()-local_received_>max_gap_))local_.reset();
  frozen_authority_=authority_;frozen_local_=local_;ready_=bool(frozen_authority_&&frozen_local_);prepared_=false;override_.clear();
  // Current costmap robot position is retained only to block the correct area
  // when authority is invalid. It never substitutes for map/odom alignment.
  if(authority_){x=authority_->map.x;y=authority_->map.y;}
  window_.update(Pose{x,y,0.,now},now);
 }
 void bounds(double resolution,double inflation,double*a,double*b,double*c,double*d)const{
  if(enabled())window_.bounds(radius_+resolution+inflation,a,b,c,d);
 }
 void apply(nav2_costmap_2d::Costmap2D& grid,int ax,int ay,int bx,int by,double now){
  if(!enabled()||!window_.current)return;
  if(ready_&&(!fresh(frozen_authority_->map.stamp,now,max_pose_age_)||!fresh(rclcpp::Time(frozen_local_->header.stamp).seconds(),now,max_age_)))ready_=false;
  if(!prepared_){
   near_local::Grid src{};
   if(ready_){
    src=view(*frozen_local_);const auto&m=frozen_authority_->map;const auto&o=frozen_authority_->odom;
    const double yaw=m.yaw-o.yaw,c=std::cos(yaw),s=std::sin(yaw);
    const double tx=m.x-c*o.x+s*o.y,ty=m.y-s*o.x-c*o.y;
    const double sx=src.x,sy=src.y;src.x=c*sx-s*sy+tx;src.y=s*sx+c*sy+ty;src.yaw+=yaw;
   }
   const auto&p=*window_.current;int lx,ly,hx,hy;const double res=grid.getResolution();
   grid.worldToMapEnforceBounds(p.x-radius_-res,p.y-radius_-res,lx,ly);
   grid.worldToMapEnforceBounds(p.x+radius_+res,p.y+radius_+res,hx,hy);
   for(int j=std::max(ay,ly);j<std::min(by,hy+1);++j)for(int i=std::max(ax,lx);i<std::min(bx,hx+1);++i){
    double wx,wy;grid.mapToWorld(i,j,wx,wy);
    if(near_local::touches_disk(wx,wy,res,p.x,p.y,radius_))override_.emplace_back(grid.getIndex(i,j),ready_?near_local::sample(src,wx,wy,res):255);
   }
   prepared_=true;
  }
  auto*data=grid.getCharMap();for(auto [index,value]:override_)data[index]=ready_?value:255;
 }
};
}
