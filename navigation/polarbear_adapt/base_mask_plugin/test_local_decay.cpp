// Isolated native Nav2 regression. No hardware subscribers or control topics.
#include "layers.cpp"
#include <rcl/time.h>
#include <nav2_util/lifecycle_node.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
struct Fixture: arx_base_mask::VoxelLayer {
  void legacy(double wx,double wy){unsigned int x,y;require(worldToMap(wx,wy,x,y),"legacy map");
    voxel_grid_.markVoxel(x,y,2);setCost(x,y,254);}
  unsigned char cost(double wx,double wy){unsigned int x,y;require(worldToMap(wx,wy,x,y),"cost map");return getCost(x,y);}
  uint32_t bits(double wx,double wy){unsigned int x,y;require(worldToMap(wx,wy,x,y),"bits map");return voxel_grid_.getData()[getIndex(x,y)];}
};
int main(int argc,char** argv){
  if(!std::getenv("ROS_DOMAIN_ID")||std::string(std::getenv("ROS_DOMAIN_ID"))!="164")return 2;
  rclcpp::init(argc,argv);
  try{
    rclcpp::NodeOptions options;options.parameter_overrides({
      rclcpp::Parameter("use_sim_time",true),
      rclcpp::Parameter("voxel.chassis_mask_file",argv[1]),
      rclcpp::Parameter("voxel.history_decay_seconds",5.),
      rclcpp::Parameter("voxel.footprint_clearing_enabled",false),
      rclcpp::Parameter("voxel.min_obstacle_height",.03),
      rclcpp::Parameter("voxel.max_obstacle_height",.4),
      rclcpp::Parameter("voxel.z_resolution",.1),
      rclcpp::Parameter("voxel.z_voxels",16),
      rclcpp::Parameter("voxel.observation_sources",std::string("")),
      rclcpp::Parameter("guard.chassis_mask_file",argv[1]),
      rclcpp::Parameter("guard.history_decay_seconds",5.),
      rclcpp::Parameter("guard.footprint_clearing_enabled",false),
      rclcpp::Parameter("guard.observation_sources",std::string("bottom")),
      rclcpp::Parameter("guard.bottom.data_type",std::string("LaserScan")),
      rclcpp::Parameter("guard.bottom.topic",std::string("/private_local_decay/no_data")),
      rclcpp::Parameter("guard.bottom.expected_update_rate",.5)});
    auto node=std::make_shared<nav2_util::LifecycleNode>("local_decay_test","",options);
    node->declare_parameter("track_unknown_space",false);
    node->declare_parameter("transform_tolerance",.3);
    auto clock=node->get_clock();
    require(rcl_enable_ros_time_override(clock->get_clock_handle())==RCL_RET_OK,"clock enable");
    auto now=[&](double sec){require(rcl_set_ros_time_override(clock->get_clock_handle(),int64_t(std::llround(sec*1e9)))==RCL_RET_OK,"clock set");};
    now(10.);tf2_ros::Buffer tf(clock);
    nav2_costmap_2d::LayeredCostmap layered("odom",false,false);layered.resizeMap(200,200,.02,-2.,-2.);
    auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    Fixture voxel;voxel.initialize(&layered,"voxel",&tf,node,group);
    auto& master=*layered.getCostmap();
    auto obs=[&](double stamp,double x,double y,double z=.23){
      sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id="odom";cloud.header.stamp=rclcpp::Time(int64_t(std::llround(stamp*1e9)));
      sensor_msgs::PointCloud2Modifier mod(cloud);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(1);
      sensor_msgs::PointCloud2Iterator<float> px(cloud,"x"),py(cloud,"y"),pz(cloud,"z");*px=x;*py=y;*pz=z;
      geometry_msgs::msg::Point origin;origin.z=.23;
      return nav2_costmap_2d::Observation(origin,cloud,8.,.35,8.,.35);
    };
    auto packet=[&](double stamp,double x,double y,double z=.23,bool marking=true,bool clearing=true){
      voxel.clearStaticObservations(true,true);auto o=obs(stamp,x,y,z);voxel.addStaticObservation(o,marking,clearing);
    };
    auto cycle=[&](double t){now(t);double a=1e9,b=1e9,c=-1e9,d=-1e9;
      voxel.updateBounds(0,0,0,&a,&b,&c,&d);master.resetMap(0,0,200,200);voxel.updateCosts(master,0,0,200,200);};
    auto mcost=[&](double x,double y){unsigned int i,j;require(master.worldToMap(x,y,i,j),"master map");return master.getCost(i,j);};
    packet(10.,1.01,.01);cycle(10.);require(voxel.cost(1.01,.01)==254,"hit not marked");
    cycle(14.99);require(voxel.cost(1.01,.01)==254,"early expiry");
    cycle(15.);require(voxel.cost(1.01,.01)==0,"cached cloud refreshed expired mark");
    require(voxel.bits(1.01,.01)==0,"expired native voxel bits remained");
    require(mcost(1.01,.01)==0,"expired FREE not merged into master");
    cycle(15.1);require(voxel.cost(1.01,.01)==0,"cached old point re-marked expired FREE");
    packet(15.2,1.01,.01);cycle(15.2);require(mcost(1.01,.01)==254,"new obstacle not restored");
    packet(15.3,1.81,.01);cycle(15.3);require(mcost(1.01,.01)==0,"fresh through did not clear early");
    cycle(20.4);require(mcost(1.01,.01)==0,"free tombstone turned back to occupied/unknown");
    require(mcost(1.81,.01)==0,"second obstacle did not expire");
    packet(15.3,1.81,.01);cycle(20.5);require(mcost(1.81,.01)==0,"stale endpoint restored");
    // Blind/occluded legacy column: a later actual ray must not claim the blind cell is free.
    voxel.clearStaticObservations(true,true);voxel.legacy(.31,.01);cycle(21.);
    packet(26.,1.81,.01);cycle(26.);require(mcost(.31,.01)==0,"blind legacy column failed to expire");
    packet(26.1,1.81,.01);cycle(26.1);require(mcost(.31,.01)==0,"blind column revived after expiry");
    // Actual origin shift moves history with odom, never with array index.
    const auto untouched=voxel.cost(.71,.41);
    voxel.updateOrigin(-1.6,-1.6);master.resizeMap(200,200,.02,voxel.getOriginX(),voxel.getOriginY());cycle(26.2);
    require(voxel.cost(.31,.01)==0,"rolling shifted cleared history to another world cell");
    require(voxel.cost(.71,.41)==untouched,"rolling created phantom unknown at old index");
    // Range/height-invalid hit cannot keep an old column alive.
    voxel.legacy(.91,.51);cycle(27.);packet(31.9,.91,.51,.5);cycle(31.9);
    cycle(32.);require(voxel.cost(.91,.51)==0,"out-of-height hit refreshed history");
    voxel.reset();cycle(33.);require(mcost(.31,.01)==0,"explicit native reset retained timestamps");
    packet(33.1,1.01,.01);
    for(int i=0;i<40;++i){const double t=33.1+i*.2;packet(t,1.01,.01);cycle(t);
      require(mcost(1.01,.01)==254,"continuous real hit expired after five seconds");}
    Fixture guard;guard.initialize(&layered,"guard",&tf,node,group);guard.legacy(.51,.51);
    double a=1e9,b=1e9,c=-1e9,d=-1e9;guard.updateBounds(0,0,0,&a,&b,&c,&d);
    now(47.);guard.updateBounds(0,0,0,&a,&b,&c,&d);
    require(guard.cost(.51,.51)==0,"outage history did not age");
    require(!guard.isCurrent(),"history decay bypassed native source outage watchdog");
    std::cout<<"PASS native Local: timeout FREE, cached packet, fresh clear/hit, master, blind range, rolling and reset\n";
  }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
