#include "startup_view_policy.hpp"
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <OgreViewport.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_rendering/render_window.hpp>

namespace arx_waypoints {
class StartupView : public rviz_common::Display {
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  std::mutex mutex_;
  geometry_msgs::msg::PoseArray::ConstSharedPtr pose_;
  float elapsed_=0.;
  bool applied_=false;
protected:
  void onInitialize() override {
    node_=context_->getRosNodeAbstraction().lock()->get_raw_node();
    subscription_=node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "/pb/chassis_pose",rclcpp::QoS(1).transient_local(),[this](geometry_msgs::msg::PoseArray::ConstSharedPtr m){
        std::lock_guard<std::mutex> lock(mutex_);pose_=m;
      });
    setStatus(rviz_common::properties::StatusProperty::Warn,"启动视角","等待当前有效车位；仅调整视角，不发送运动指令");
  }
  void update(float wall_dt,float) override {
    if(applied_||!isEnabled())return;
    elapsed_+=wall_dt;
    if(elapsed_<1.)return; // Let the restored window and its dock layout settle.
    geometry_msgs::msg::PoseArray::ConstSharedPtr pose;
    {std::lock_guard<std::mutex> lock(mutex_);pose=pose_;}
    if(!pose||pose->header.frame_id!="map"||pose->poses.size()!=2||context_->getFixedFrame()!="map")return;
    auto *manager=context_->getViewManager();auto *view=manager->getCurrent();
    if(!view||view->getClassId()!="rviz_default_plugins/TopDownOrtho")return;
    auto *window=manager->getRenderPanel()->getRenderWindow();
    auto *viewport=rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(window);
    if(!viewport)return;
    const auto &p=pose->poses[0].position;
    const double age=(node_->now()-rclcpp::Time(pose->header.stamp)).seconds();
    auto camera=startup_camera(p.x,p.y,age,viewport->getActualWidth());
    if(!camera)return;
    view->subProp("X")->setValue(camera->x);
    view->subProp("Y")->setValue(camera->y);
    view->subProp("Scale")->setValue(camera->scale);
    applied_=true;
    setStatus(rviz_common::properties::StatusProperty::Ok,"启动视角","已对准当前车位；横向覆盖实际10米；可继续手动缩放和平移");
    RCLCPP_INFO(node_->get_logger(),"Startup camera centered x=%.3f y=%.3f, width=%upx, scale=%.3fpx/m, horizontal_span=10m",camera->x,camera->y,viewport->getActualWidth(),camera->scale);
    subscription_.reset();
  }
};
}
PLUGINLIB_EXPORT_CLASS(arx_waypoints::StartupView,rviz_common::Display)
