#include <cstdlib>
#include <iostream>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/voxel_layer.hpp>
#include <pluginlib/class_loader.hpp>
int main(){
  if(!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID"))!="198")return 2;
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d","nav2_costmap_2d::Layer");
  auto a=loader.createSharedInstance("arx_base_mask::InflationLayer");
  auto b=loader.createSharedInstance("arx_base_mask::VoxelLayer");
  if(!std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(a) ||
     !std::dynamic_pointer_cast<nav2_costmap_2d::VoxelLayer>(b))return 1;
  std::cout<<"PASS: native subclass plugins load via installed registry\n";
}
