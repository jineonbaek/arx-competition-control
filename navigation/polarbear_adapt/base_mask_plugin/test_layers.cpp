// Real native-layer integration, private domain only, no sensor/actuator topics.
#include <cstdlib>
#include <iostream>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/voxel_layer.hpp>
#include <tf2_ros/buffer.h>
#ifdef BASELINE
namespace subject {
using Inflation=nav2_costmap_2d::InflationLayer;
using Voxel=nav2_costmap_2d::VoxelLayer;
}
#else
#include "layers.cpp"
namespace subject {using Inflation=arx_base_mask::InflationLayer;using Voxel=arx_base_mask::VoxelLayer;}
#endif
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
struct VoxelFixture:subject::Voxel {
  void mark(unsigned int x,unsigned int y){voxel_grid_.markVoxel(x,y,5);setCost(x,y,254);}
  uint32_t column(unsigned int x,unsigned int y){return voxel_grid_.getData()[getIndex(x,y)];}
};
void pose(tf2_ros::Buffer& tf,rclcpp::Clock::SharedPtr clock,double x,double y,double yaw){
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id="map";t.child_frame_id="base_link";t.header.stamp=clock->now();
  t.transform.translation.x=x;t.transform.translation.y=y;
  t.transform.rotation.z=std::sin(yaw/2);t.transform.rotation.w=std::cos(yaw/2);
  tf.setTransform(t,"offline_fixture",false);
}
int main(int argc,char** argv){
  if(!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID"))!="198")return 2;
  rclcpp::init(argc,argv);
  try{
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("inflation.chassis_mask_file",argv[1]),
      rclcpp::Parameter("voxel.chassis_mask_file",argv[1]),
      rclcpp::Parameter("inflation.inflation_radius",.5),
      rclcpp::Parameter("voxel.footprint_clearing_enabled",false),
      rclcpp::Parameter("voxel.observation_sources",std::string(""))});
    auto node=std::make_shared<nav2_util::LifecycleNode>("base_mask_offline_test","",options);
    auto clock=node->get_clock();tf2_ros::Buffer tf(clock);
    nav2_costmap_2d::LayeredCostmap layered("map",false,false);
    layered.resizeMap(80,80,.05,-2.,-2.);
    std::vector<geometry_msgs::msg::Point> footprint;
    for(auto xy:{std::pair<double,double>{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}}){
      geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);}
    layered.setFootprint(footprint);
    auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subject::Inflation inflation;
    inflation.initialize(&layered,"inflation",&tf,node,group);
    auto& grid=*layered.getCostmap();
    unsigned int inside_x,inside_y,outside_x,outside_y;
    grid.worldToMap(0,0,inside_x,inside_y);grid.worldToMap(1.,1.,outside_x,outside_y);
    auto normal=[&](){grid.resetMap(0,0,80,80);grid.setCost(inside_x,inside_y,254);
                     grid.setCost(inside_x+1,inside_y,255);grid.setCost(outside_x,outside_y,254);};
    auto cycle=[&](){double a=0,b=0,c=0,d=0;
      inflation.updateBounds(0,0,0,&a,&b,&c,&d);inflation.updateCosts(grid,0,0,80,80);};
    pose(tf,clock,0,0,0);normal();cycle();
    require(grid.getCost(inside_x,inside_y)==0,"chassis interior remained lethal after native inflation");
    require(grid.getCost(inside_x+1,inside_y)==0,"chassis unknown cell not free");
    require(grid.getCost(outside_x,outside_y)==254,"outside cone was cleared");
    require(grid.getCost(inside_x-7,inside_y)==0,"internal seed inflated outside chassis");
    require(grid.getCost(outside_x+1,outside_y)>0,"outside cone inflation disappeared");
    pose(tf,clock,.8,0,1.5707963267948966);normal();cycle();
    require(grid.getCost(inside_x,inside_y)==254,"old region did not return to normal source value");
    tf.clear();normal();cycle();
    require(grid.getCost(inside_x,inside_y)==254,"missing TF still cleared map");
    VoxelFixture voxel;
    voxel.initialize(&layered,"voxel",&tf,node,group);
    voxel.mark(inside_x,inside_y);voxel.mark(outside_x,outside_y);
    pose(tf,clock,0,0,0);double a=0,b=0,c=0,d=0;
    voxel.updateBounds(0,0,0,&a,&b,&c,&d);
    require(voxel.getCost(inside_x,inside_y)==0 && voxel.column(inside_x,inside_y)==0,
            "local history column remained occupied inside chassis");
    require(voxel.getCost(outside_x,outside_y)==254 && voxel.column(outside_x,outside_y)!=0,
            "outside local history column was cleared");
    voxel.updateOrigin(-1.8,-1.7);
    voxel.worldToMap(0,0,inside_x,inside_y);voxel.worldToMap(1,1,outside_x,outside_y);
    voxel.mark(inside_x,inside_y);voxel.mark(outside_x,outside_y);
    pose(tf,clock,0,0,0);voxel.updateBounds(0,0,0,&a,&b,&c,&d);
    require(voxel.column(inside_x,inside_y)==0 && voxel.getCost(inside_x,inside_y)==0,
            "rolling origin used wrong chassis indices");
    require(voxel.column(outside_x,outside_y)!=0 && voxel.getCost(outside_x,outside_y)==254,
            "rolling origin cleared outside history");
    std::cout<<"PASS: native inflation and voxel integration; outside cone and history retained\n";
  }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
