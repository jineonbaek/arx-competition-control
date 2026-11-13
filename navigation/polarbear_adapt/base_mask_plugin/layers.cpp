#include "geometry.hpp"
#include <fstream>
#include <iterator>
#include <memory>
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
 public:
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node,const std::string& name) {
    const auto parameter=name+".chassis_mask_file";
    const auto file=ament_index_cpp::get_package_share_directory("arx_base_mask_costmap")+"/base_envelope.wkt";
    if(!node->has_parameter(parameter))node->declare_parameter(parameter,file);
    std::ifstream input(node->get_parameter(parameter).as_string());
    if(!input)throw std::runtime_error("Cannot read chassis envelope");
    geometry_=std::make_unique<Geometry>(std::string(std::istreambuf_iterator<char>(input),{}));
  }
  void update(nav2_costmap_2d::LayeredCostmap* layered,tf2_ros::Buffer* tf,
              const rclcpp::Clock::SharedPtr& clock,bool enabled) {
    std::optional<Pose> pose;
    if(enabled)try {
      const auto t=tf->lookupTransform(layered->getGlobalFrameID(),"base_link",tf2::TimePointZero);
      const auto& q=t.transform.rotation;
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(std::isfinite(norm)&&std::abs(norm-1.)<.02)
        pose=Pose{t.transform.translation.x,t.transform.translation.y,tf2::getYaw(q),
                  rclcpp::Time(t.header.stamp,clock->get_clock_type()).seconds()};
    }catch(const tf2::TransformException&){}
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
 public:
  void onInitialize() override {
    nav2_costmap_2d::InflationLayer::onInitialize();mask_.initialize(node_.lock(),name_);
  }
  void updateBounds(double x,double y,double yaw,double* a,double* b,double* c,double* d) override {
    mask_.update(layered_costmap_,tf_,clock_,enabled_);
    mask_.bounds(layered_costmap_->getCostmap()->getResolution(),a,b,c,d);
    nav2_costmap_2d::InflationLayer::updateBounds(x,y,yaw,a,b,c,d);
  }
  void updateCosts(nav2_costmap_2d::Costmap2D& grid,int a,int b,int c,int d) override {
    auto free=[&](int x,int y){grid.setCost(x,y,nav2_costmap_2d::FREE_SPACE);};
    // Remove internal seeds before convolution, then exclude external inflation
    // from the current chassis interior. Outside cells retain native Nav2 costs.
    if(enabled_)mask_.cells(grid,a,b,c,d,clock_->now().seconds(),free);
    nav2_costmap_2d::InflationLayer::updateCosts(grid,a,b,c,d);
    if(enabled_)mask_.cells(grid,a,b,c,d,clock_->now().seconds(),free);
  }
};
class VoxelLayer : public nav2_costmap_2d::VoxelLayer {
  Mask mask_;
 public:
  void onInitialize() override {
    nav2_costmap_2d::VoxelLayer::onInitialize();mask_.initialize(node_.lock(),name_);
  }
  void updateBounds(double x,double y,double yaw,double* a,double* b,double* c,double* d) override {
    nav2_costmap_2d::VoxelLayer::updateBounds(x,y,yaw,a,b,c,d);
    mask_.update(layered_costmap_,tf_,clock_,enabled_);
    mask_.bounds(getResolution(),a,b,c,d);
    if(enabled_)mask_.cells(*this,0,0,getSizeInCellsX(),getSizeInCellsY(),clock_->now().seconds(),
      [&](int i,int j){voxel_grid_.clearVoxelColumn(getIndex(i,j));setCost(i,j,nav2_costmap_2d::FREE_SPACE);});
  }
};
}
PLUGINLIB_EXPORT_CLASS(arx_base_mask::InflationLayer,nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(arx_base_mask::VoxelLayer,nav2_costmap_2d::Layer)
