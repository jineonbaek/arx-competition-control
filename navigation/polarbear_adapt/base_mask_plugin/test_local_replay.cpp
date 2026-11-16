// Feed measured, source-stamped finite scans into old native and candidate side by side.
#include "layers.cpp"
#include <rcl/time.h>
#include <nav2_util/lifecycle_node.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cstdlib>
struct ReplayVoxel:arx_base_mask::VoxelLayer {
 void seed(const std::vector<unsigned char>& data){for(unsigned int i=0;i<data.size();++i)if(data[i]==254){
   unsigned int x,y;indexToCells(i,x,y);voxel_grid_.markVoxel(x,y,2);setCost(x,y,254);}}
};
int main(int argc,char** argv){
 if(!std::getenv("ROS_DOMAIN_ID")||std::string(std::getenv("ROS_DOMAIN_ID"))!="164"||argc<3)return 2;
 rclcpp::init(argc,argv);
 try{
  std::ifstream in(argv[1]);char tag;double first,res,ox,oy;unsigned int width,height;
  in>>tag>>first>>width>>height>>res>>ox>>oy;if(tag!='G')throw std::runtime_error("replay header");
  std::vector<unsigned char> cells(width*height);for(auto& p:cells){int v;in>>v;p=v;}
  rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("use_sim_time",true),
   rclcpp::Parameter("baseline.chassis_mask_file",argv[2]),rclcpp::Parameter("candidate.chassis_mask_file",argv[2]),
   rclcpp::Parameter("candidate.history_decay_seconds",5.)});
  auto node=std::make_shared<nav2_util::LifecycleNode>("local_decay_measured_replay","",options);
  node->declare_parameter("track_unknown_space",false);node->declare_parameter("transform_tolerance",.3);
  auto clock=node->get_clock();rcl_enable_ros_time_override(clock->get_clock_handle());
  auto now=[&](double t){rcl_set_ros_time_override(clock->get_clock_handle(),int64_t(std::llround(t*1e9)));};
  now(first);tf2_ros::Buffer tf(clock);
  nav2_costmap_2d::LayeredCostmap layered("odom",false,false);layered.resizeMap(width,height,res,ox,oy);
  auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  for(const auto& name:{"baseline","candidate"}){
   node->declare_parameter(std::string(name)+".observation_sources",std::string(""));
   node->declare_parameter(std::string(name)+".footprint_clearing_enabled",false);
   node->declare_parameter(std::string(name)+".min_obstacle_height",.03);
   node->declare_parameter(std::string(name)+".max_obstacle_height",.4);
   node->declare_parameter(std::string(name)+".z_resolution",.1);
   node->declare_parameter(std::string(name)+".z_voxels",16);
  }
  ReplayVoxel baseline,candidate;baseline.initialize(&layered,"baseline",&tf,node,group);candidate.initialize(&layered,"candidate",&tf,node,group);
  baseline.seed(cells);candidate.seed(cells);
  std::unique_ptr<nav2_costmap_2d::Observation> latest[2];unsigned int count=0;double elapsed=0.,max_ms=0.,total_ms=0.;
  std::vector<double> last_hit(cells.size(),-1.);
  while(in>>tag){
   double received,stamp;int sensor;geometry_msgs::msg::Point origin;unsigned int n;
   in>>received>>sensor>>stamp>>origin.x>>origin.y>>origin.z>>n;
   if(tag!='P'||sensor<0||sensor>1)throw std::runtime_error("packet header");
   sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id="odom";cloud.header.stamp=rclcpp::Time(int64_t(std::llround(stamp*1e9)));
   sensor_msgs::PointCloud2Modifier mod(cloud);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(n);
   sensor_msgs::PointCloud2Iterator<float> px(cloud,"x"),py(cloud,"y"),pz(cloud,"z");
   for(unsigned int j=0;j<n;++j,++px,++py,++pz){double x,y,z;in>>x>>y>>z;*px=x;*py=y;*pz=z;
    unsigned int mx,my;const double sq=(x-origin.x)*(x-origin.x)+(y-origin.y)*(y-origin.y)+(z-origin.z)*(z-origin.z);
    if(z>=.03&&z<=.4&&sq>=.35*.35&&sq<64.&&candidate.worldToMap(x,y,mx,my))last_hit[candidate.getIndex(mx,my)]=stamp;
   }
   latest[sensor]=std::make_unique<nav2_costmap_2d::Observation>(origin,cloud,8.,.35,8.,.35);now(received);
   for(auto layer:{&baseline,&candidate}){
    layer->clearStaticObservations(true,true);for(const auto& o:latest)if(o)layer->addStaticObservation(*o,true,true);
    double a=1e9,b=1e9,c=-1e9,d=-1e9;auto begin=std::chrono::steady_clock::now();layer->updateBounds(0,0,0,&a,&b,&c,&d);
    if(layer==&candidate){double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();total_ms+=ms;max_ms=std::max(max_ms,ms);}
   }
   elapsed=received-first;++count;
  }
  unsigned int old_lethal=0,new_lethal=0,expired=0,recent=0,recent_kept=0;
  std::ofstream diff(std::string(argv[1])+".cells.csv");diff<<"x,y,initial,baseline,candidate,last_hit_age_s\n";
  for(unsigned int i=0;i<cells.size();++i){
   auto old=baseline.getCharMap()[i],next=candidate.getCharMap()[i];old_lethal+=old==254;new_lethal+=next==254;
   if(old==254&&next==0)++expired;
   if(old==254&&last_hit[i]>0&&first+elapsed-last_hit[i]<.65){++recent;recent_kept+=next==254;}
   if(old!=next||cells[i]==254){unsigned int x,y;candidate.indexToCells(i,x,y);double wx,wy;candidate.mapToWorld(x,y,wx,wy);
    diff<<wx<<','<<wy<<','<<int(cells[i])<<','<<int(old)<<','<<int(next)<<','<<(last_hit[i]<0 ? -1. : first+elapsed-last_hit[i])<<'\n';}
  }
  std::cout<<"{\"packets\":"<<count<<",\"duration_s\":"<<elapsed<<",\"baseline_lethal\":"<<old_lethal<<",\"candidate_lethal\":"<<new_lethal
   <<",\"baseline_lethal_now_free\":"<<expired<<",\"recent_hit_columns\":"<<recent<<",\"recent_hit_retained\":"<<recent_kept
   <<",\"candidate_mean_ms\":"<<total_ms/count<<",\"candidate_max_ms\":"<<max_ms<<"}\n";
  if(recent!=recent_kept)throw std::runtime_error("recent real obstacle lost");
 }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
