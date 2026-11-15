#include "geometry.hpp"
#include "near_local_layer.hpp"
#include "local_decay_layer.hpp"
#include "escape_sparse_layer.hpp"
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/voxel_layer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arx_base_mask {
class Mask {
  std::unique_ptr<Geometry> geometry_;
  Window window_;
  std::mutex mutex_;
  std::optional<Pose> latest_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  std::string frame_;
  double floor_=-1.,last_now_=-1.;
  void withdraw() {if(latest_)floor_=std::max(floor_,latest_->stamp);latest_.reset();}
 public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node,const std::string& name,const std::string& frame) {
    frame_=frame;
    const auto parameter=name+".chassis_mask_file";
    const auto file=ament_index_cpp::get_package_share_directory("arx_base_mask_costmap")+"/base_envelope.wkt";
    if(!node->has_parameter(parameter))node->declare_parameter(parameter,file);
    std::ifstream input(node->get_parameter(parameter).as_string());
    if(!input)throw std::runtime_error("Cannot read chassis envelope");
    geometry_=std::make_unique<Geometry>(std::string(std::istreambuf_iterator<char>(input),{}));
    if(frame_!="map" && frame_!="odom")throw std::runtime_error("Unsupported chassis mask frame");
    subscription_=node->create_subscription<geometry_msgs::msg::PoseArray>("/pb/chassis_pose",
      rclcpp::QoS(1).transient_local(),[this](geometry_msgs::msg::PoseArray::ConstSharedPtr m){snapshot(*m);});
  }
  void snapshot(const geometry_msgs::msg::PoseArray& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(m.header.frame_id!="map" || m.poses.size()!=2){withdraw();return;}
    const double stamp=rclcpp::Time(m.header.stamp).seconds();
    if(stamp<=floor_){withdraw();return;}
    for(const auto& p:m.poses){
      const auto& q=p.orientation;const auto& t=p.position;
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(!std::isfinite(t.x)||!std::isfinite(t.y)||!std::isfinite(t.z)||
         !std::isfinite(norm)||std::abs(norm-1.)>=.02){withdraw();return;}
    }
    const auto& p=m.poses[frame_=="map" ? 0 : 1];
    latest_=Pose{p.position.x,p.position.y,tf2::getYaw(p.orientation),
                  rclcpp::Time(m.header.stamp).seconds()};
  }
  void update(nav2_costmap_2d::LayeredCostmap* layered,tf2_ros::Buffer* tf,
              const rclcpp::Clock::SharedPtr& clock,bool enabled) {
    std::optional<Pose> pose;
    {std::lock_guard<std::mutex> lock(mutex_);
      const double now=clock->now().seconds();
      if((last_now_>=0 && now<last_now_) || (latest_ && !latest_->fresh(now)))withdraw();
      last_now_=now;if(enabled)pose=latest_;}
    window_.update(pose,clock->now().seconds());
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("chassis_mask_diagnostic"),*clock,5000,
      "frame=%s valid=%d age=%.6f x=%.3f y=%.3f",layered->getGlobalFrameID().c_str(),
      bool(window_.current),pose ? clock->now().seconds()-pose->stamp : -999.,
      pose ? pose->x : 0.,pose ? pose->y : 0.);
  }
  void bounds(double resolution,double* a,double* b,double* c,double* d) const {
    window_.bounds(geometry_->radius+resolution,a,b,c,d);
  }
  template<class Function>
  void cells(nav2_costmap_2d::Costmap2D& grid,int lo_x,int lo_y,int hi_x,int hi_y,
             double now,Function apply) const {
    if(!window_.current || !window_.current->fresh(now))return;
    const auto& p=*window_.current;int a,b,c,d;
    grid.worldToMapEnforceBounds(p.x-geometry_->radius,p.y-geometry_->radius,a,b);
    grid.worldToMapEnforceBounds(p.x+geometry_->radius,p.y+geometry_->radius,c,d);
    for(int y=std::max(b,lo_y);y<std::min(d+1,hi_y);++y)
      for(int x=std::max(a,lo_x);x<std::min(c+1,hi_x);++x) {
        double wx,wy;grid.mapToWorld(x,y,wx,wy);
        if(geometry_->covers(wx,wy,grid.getResolution(),p))apply(x,y);
      }
  }
};

class InflationLayer : public nav2_costmap_2d::InflationLayer {
  Mask mask_;
  NearLocal near_;
  EscapeSparse sparse_;
 public:
  void onInitialize() override {
    nav2_costmap_2d::InflationLayer::onInitialize();mask_.initialize(node_.lock(),name_,layered_costmap_->getGlobalFrameID());
    near_.initialize(node_.lock(),name_,layered_costmap_->getGlobalFrameID());
    sparse_.initialize(node_.lock(),name_,layered_costmap_->getGlobalFrameID());
  }
  void updateBounds(double x,double y,double yaw,double* a,double* b,double* c,double* d) override {
    mask_.update(layered_costmap_,tf_,clock_,enabled_);
    mask_.bounds(layered_costmap_->getCostmap()->getResolution(),a,b,c,d);
    sparse_.update(clock_->now().seconds());
    sparse_.bounds(inflation_radius_+layered_costmap_->getCostmap()->getResolution(),a,b,c,d);
    near_.update(x,y,clock_->now().seconds());
    near_.bounds(layered_costmap_->getCostmap()->getResolution(),inflation_radius_,a,b,c,d);
    nav2_costmap_2d::InflationLayer::updateBounds(x,y,yaw,a,b,c,d);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D& grid,int a,int b,int c,int d) override {
    auto free=[&](int x,int y){grid.setCost(x,y,nav2_costmap_2d::FREE_SPACE);};
    // Remove internal seeds before convolution, then exclude external inflation
    // from the current chassis interior. Outside cells retain native Nav2 costs.
    if(enabled_)mask_.cells(grid,a,b,c,d,clock_->now().seconds(),free);
    if(enabled_)sparse_.apply(grid,clock_);
    if(enabled_)near_.apply(grid,a,b,c,d,clock_->now().seconds());
    nav2_costmap_2d::InflationLayer::updateCosts(grid,a,b,c,d);
    if(enabled_)mask_.cells(grid,a,b,c,d,clock_->now().seconds(),free);
    if(enabled_)near_.apply(grid,a,b,c,d,clock_->now().seconds());
    if(near_.enabled())current_=near_.ready();
  }
};
class VoxelLayer : public nav2_costmap_2d::VoxelLayer {
  Mask mask_;
  LocalDecay decay_;
 protected:
  void raytraceFreespace(const nav2_costmap_2d::Observation& obs,
      double* a,double* b,double* c,double* d) override {
    nav2_costmap_2d::VoxelLayer::raytraceFreespace(obs,a,b,c,d);
    const double now=clock_->now().seconds(),stamp=rclcpp::Time(obs.cloud_->header.stamp).seconds();
    if(!decay_.fresh(stamp,now))return;
    unsigned int sx,sy,sz;
    if(!worldToMap3D(obs.origin_.x,obs.origin_.y,obs.origin_.z,sx,sy,sz))return;
    sensor_msgs::PointCloud2ConstIterator<float> px(*obs.cloud_,"x"),py(*obs.cloud_,"y"),pz(*obs.cloud_,"z");
    const double ox=obs.origin_.x,oy=obs.origin_.y,oz=obs.origin_.z;
    for(;px!=px.end();++px,++py,++pz){
      const double dx=*px-ox,dy=*py-oy,dz=*pz-oz;
      const double distance=std::sqrt(dx*dx+dy*dy+dz*dz);
      if(!std::isfinite(distance)||distance<=2*getResolution())continue;
      // Match native endpoint margin, max range, and clip strictly inside grid.
      double t=std::min({1.-2*getResolution()/distance,obs.raytrace_max_range_/distance,1.});
      const double start[]={ox,oy,oz},delta[]={dx,dy,dz};
      const double lo[]={getOriginX(),getOriginY(),origin_z_};
      const double hi[]={getOriginX()+getSizeInCellsX()*getResolution(),
                         getOriginY()+getSizeInCellsY()*getResolution(),origin_z_+voxel_grid_.sizeZ()*z_resolution_};
      for(int axis=0;axis<3;++axis){
        if(delta[axis]>0.)t=std::min(t,(hi[axis]-1e-6-start[axis])/delta[axis]);
        else if(delta[axis]<0.)t=std::min(t,(lo[axis]+1e-6-start[axis])/delta[axis]);
      }
      if(t<=0.)continue;
      unsigned int ex,ey,ez;
      if(!worldToMap3D(ox+t*dx,oy+t*dy,oz+t*dz,ex,ey,ez))continue;
      raytraceLine([&](unsigned int index){
        unsigned int x,y;indexToCells(index,x,y);double wx,wy;mapToWorld(x,y,wx,wy);
        // Do not convert a partly blind grid cell to free by rounding min range.
        if(std::hypot(wx-ox,wy-oy)>=obs.raytrace_min_range_+getResolution()/std::sqrt(2.))
          decay_.crossed(x,y,stamp);
      },sx,sy,ex,ey);
    }
  }
 public:
  void onInitialize() override {
    nav2_costmap_2d::VoxelLayer::onInitialize();mask_.initialize(node_.lock(),name_,layered_costmap_->getGlobalFrameID());
    decay_.initialize(node_.lock(),name_);
  }
  void matchSize() override {nav2_costmap_2d::VoxelLayer::matchSize();decay_.reset();}
  void reset() override {nav2_costmap_2d::VoxelLayer::reset();decay_.reset();}
  void updateBounds(double x,double y,double yaw,double* a,double* b,double* c,double* d) override {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
    const double now=clock_->now().seconds();
    decay_.grid(*this);decay_.begin(now);
    // Native will shift the rolling origin before invoking raytraceFreespace.
    if(rolling_window_)updateOrigin(x-getSizeInMetersX()/2,y-getSizeInMetersY()/2);
    decay_.grid(*this);
    nav2_costmap_2d::VoxelLayer::updateBounds(x,y,yaw,a,b,c,d);
    if(enabled_&&decay_.enabled()){
      std::vector<nav2_costmap_2d::Observation> observations;getMarkingObservations(observations);
      for(const auto& obs:observations){
        const double stamp=rclcpp::Time(obs.cloud_->header.stamp).seconds();
        if(!decay_.fresh(stamp,now))continue;
        sensor_msgs::PointCloud2ConstIterator<float> px(*obs.cloud_,"x"),py(*obs.cloud_,"y"),pz(*obs.cloud_,"z");
        for(;px!=px.end();++px,++py,++pz){
          if(!std::isfinite(*px)||!std::isfinite(*py)||!std::isfinite(*pz)||*pz<min_obstacle_height_||*pz>max_obstacle_height_)continue;
          const double dx=*px-obs.origin_.x,dy=*py-obs.origin_.y,dz=*pz-obs.origin_.z;
          const double squared=dx*dx+dy*dy+dz*dz;
          if(squared<obs.obstacle_min_range_*obs.obstacle_min_range_||squared>=obs.obstacle_max_range_*obs.obstacle_max_range_)continue;
          unsigned int i,j,k;
          if(worldToMap3D(*px,*py,std::max(double(*pz),origin_z_),i,j,k)&&getCost(i,j)==nav2_costmap_2d::LETHAL_OBSTACLE)
            decay_.hit(i,j,stamp,now);
        }
      }
      decay_.adopt(*this,now);
      decay_.apply(*this,now,[&](unsigned int i,unsigned int j,unsigned char cost){
        voxel_grid_.clearVoxelColumn(getIndex(i,j));setCost(i,j,cost);
      },a,b,c,d);
    }
    mask_.update(layered_costmap_,tf_,clock_,enabled_);
    mask_.bounds(getResolution(),a,b,c,d);
    if(enabled_)mask_.cells(*this,0,0,getSizeInCellsX(),getSizeInCellsY(),now,
      [&](int i,int j){voxel_grid_.clearVoxelColumn(getIndex(i,j));setCost(i,j,nav2_costmap_2d::FREE_SPACE);decay_.forget(i,j);});
  }
  void updateCosts(nav2_costmap_2d::Costmap2D& grid,int a,int b,int c,int d) override {
    nav2_costmap_2d::VoxelLayer::updateCosts(grid,a,b,c,d);
    if(enabled_)decay_.mergeUnknown(grid,a,b,c,d);
  }
};
}
PLUGINLIB_EXPORT_CLASS(arx_base_mask::InflationLayer,nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(arx_base_mask::VoxelLayer,nav2_costmap_2d::Layer)
