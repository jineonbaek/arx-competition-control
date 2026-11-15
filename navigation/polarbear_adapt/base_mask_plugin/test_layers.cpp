// Real native-layer integration, private domain only, no sensor/actuator topics.
#include <cstdlib>
#include <iostream>
#include <thread>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rcl/time.h>
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
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
void send_snapshot(geometry_msgs::msg::PoseArray m){
  publisher->publish(m);
  for(int i=0;i<8;++i){executor->spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
}
void pose(tf2_ros::Buffer& tf,rclcpp::Clock::SharedPtr clock,double x,double y,double yaw){
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id="map";t.child_frame_id="base_link";t.header.stamp=clock->now();
  t.transform.translation.x=x;t.transform.translation.y=y;
  t.transform.rotation.z=std::sin(yaw/2);t.transform.rotation.w=std::cos(yaw/2);
  tf.setTransform(t,"offline_fixture",false);
  geometry_msgs::msg::PoseArray m;m.header.frame_id="map";m.header.stamp=t.header.stamp;
  geometry_msgs::msg::Pose p;p.position.x=x;p.position.y=y;p.orientation=t.transform.rotation;
  m.poses={p,p};send_snapshot(m);
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
    executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node->get_node_base_interface());
    publisher=node->create_publisher<geometry_msgs::msg::PoseArray>("/pb/chassis_pose",rclcpp::QoS(1).transient_local());
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
    tf.clear();geometry_msgs::msg::PoseArray invalid;invalid.header.frame_id="map";invalid.header.stamp=clock->now();
    send_snapshot(invalid);normal();cycle();
    require(grid.getCost(inside_x,inside_y)==254,"authority invalid snapshot still cleared map");
    geometry_msgs::msg::TransformStamped late;late.header.frame_id="map";late.child_frame_id="base_link";
    late.header.stamp=clock->now();late.transform.rotation.w=1.;tf.setTransform(late,"fresh_tf_is_not_authority",false);
    normal();cycle();require(grid.getCost(inside_x,inside_y)==254,"independent TF bypassed invalid authority");
    pose(tf,clock,0,0,0);normal();cycle();
    std::this_thread::sleep_for(std::chrono::milliseconds(670));normal();cycle();
    require(grid.getCost(inside_x,inside_y)==254,"dead authority/source retained free mask");
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
#ifndef BASELINE
    // Real consumer with simulated source/clock: expired cache must not revive.
    arx_base_mask::Mask cache;cache.initialize(node,"inflation","map");
    auto simulated=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    require(rcl_enable_ros_time_override(simulated->get_clock_handle())==RCL_RET_OK,"clock enable");
    geometry_msgs::msg::PoseArray cached;cached.header.frame_id="map";
    cached.header.stamp=rclcpp::Time(int64_t(10000000000));geometry_msgs::msg::Pose origin;
    origin.orientation.w=1.;cached.poses={origin,origin};send_snapshot(cached);
    auto check_cache=[&](int64_t now){
      require(rcl_set_ros_time_override(simulated->get_clock_handle(),now)==RCL_RET_OK,"clock set");
      cache.update(&layered,&tf,simulated,true);int count=0;
      cache.cells(grid,0,0,80,80,now*1e-9,[&](int,int){++count;});return count;
    };
    require(check_cache(10400000000)>0,"fresh cache not active");
    require(check_cache(10700000000)==0,"expired cache active");
    require(check_cache(10400000000)==0,"expired cache revived after rollback");
    send_snapshot(cached);require(check_cache(10400000000)==0,"replayed expired source revived");
    cached.header.stamp=rclcpp::Time(int64_t(10100000000));send_snapshot(cached);
    require(check_cache(10400000000)>0,"new source did not restore cache");
#endif
    publisher.reset();executor.reset();
    std::cout<<"PASS: native inflation and voxel integration; outside cone and history retained\n";
  }catch(const std::exception& e){publisher.reset();executor.reset();std::cerr<<"FAIL: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
