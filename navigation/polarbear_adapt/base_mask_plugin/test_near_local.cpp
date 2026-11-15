#include <iostream>
#include <thread>
#include <chrono>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "layers.cpp"
using namespace std::chrono_literals;
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(int argc,char**argv){
 if(!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID"))!="166")return 2;
 rclcpp::init(argc,argv);
 try{
  rclcpp::NodeOptions opts;opts.parameter_overrides({rclcpp::Parameter("inflation.chassis_mask_file",argv[1]),rclcpp::Parameter("inflation.near_local_radius",2.),rclcpp::Parameter("inflation.inflation_radius",.5)});
  auto node=std::make_shared<nav2_util::LifecycleNode>("near_local_native_test","",opts);auto clock=node->get_clock();tf2_ros::Buffer tf(clock);
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node->get_node_base_interface());
  auto poses=node->create_publisher<geometry_msgs::msg::PoseArray>("/pb/chassis_pose",rclcpp::QoS(1).transient_local());
  auto local=node->create_publisher<nav2_msgs::msg::Costmap>("/local_costmap/costmap_raw",rclcpp::QoS(1).transient_local());
  poses->on_activate();local->on_activate();
  nav2_costmap_2d::LayeredCostmap layered("map",false,false);layered.resizeMap(400,400,.02,-4.,-4.);
  std::vector<geometry_msgs::msg::Point> footprint;for(auto xy:{std::pair<double,double>{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}}){geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);}layered.setFootprint(footprint);
  arx_base_mask::InflationLayer layer;layer.initialize(&layered,"inflation",&tf,node,node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));auto& grid=*layered.getCostmap();
  auto drain=[&](){for(int i=0;i<5;++i){executor.spin_some();std::this_thread::sleep_for(10ms);}};
  auto pose=[&](double mx=0.,double my=0.,double oyaw=0.){geometry_msgs::msg::PoseArray m;m.header.frame_id="map";m.header.stamp=clock->now();geometry_msgs::msg::Pose a,b;a.position.x=b.position.x=mx;a.position.y=b.position.y=my;a.orientation.w=1.;b.orientation.z=std::sin(oyaw/2);b.orientation.w=std::cos(oyaw/2);m.poses={a,b};poses->publish(m);drain();};
  auto local_grid=[&](double ox=-5.,double oy=-5.,bool hit=true){nav2_msgs::msg::Costmap m;m.header.frame_id="odom";m.header.stamp=clock->now();m.metadata.size_x=m.metadata.size_y=500;m.metadata.resolution=.02;m.metadata.origin.position.x=ox;m.metadata.origin.position.y=oy;m.metadata.origin.orientation.w=1.;m.data.resize(250000,0);if(hit){unsigned i=(1.-ox)/.02,j=(1.-oy)/.02;m.data[j*500+i]=254;m.data[j*500+i+1]=120;}local->publish(m);drain();return m;};
  auto cost=[&](double x,double y){unsigned i,j;require(grid.worldToMap(x,y,i,j),"test point out of grid");return grid.getCost(i,j);};
  auto cycle=[&](double x=0.,double y=0.){grid.resetMap(0,0,400,400);unsigned i,j;for(auto p:{std::pair<double,double>{1.,0.},{3.,0.},{0.,0.}}){grid.worldToMap(p.first,p.second,i,j);grid.setCost(i,j,254);}double a=0,b=0,c=0,d=0;layer.updateBounds(x,y,0,&a,&b,&c,&d);layer.updateCosts(grid,0,0,400,400);};
  pose();auto first=local_grid();cycle();
  require(cost(1.,0.)==0,"Global near historical obstacle still vetoed Local free");
  require(cost(1.3,0.)==0,"Global near ghost inflation survived");
  require(cost(1.,1.)==254,"Local real obstacle lost in Global near field");
  require(cost(1.03,1.)==120,"Local raw inflation cost was not preserved");
  require(cost(3.,0.)==254,"far Global obstacle cleared");require(layer.isCurrent(),"fresh near field not current");
  pose();local_grid(-4.98,-5.02);cycle();require(cost(1.,1.)==254,"rolling Local origin misaligned");
  std::this_thread::sleep_for(600ms);pose();cycle();require(cost(1.,0.)==255&&!layer.isCurrent(),"stale Local fell back to Global or free");
  local->publish(first);drain();pose();cycle();require(cost(1.,0.)==255,"old Local snapshot revived");
  local_grid();pose();cycle();require(cost(1.,0.)==0&&layer.isCurrent(),"fresh source did not recover");
  pose();local_grid();std::this_thread::sleep_for(700ms);local_grid();cycle();require(cost(1.,0.)==255&&!layer.isCurrent(),"stale authority allowed fresh Local to bypass pause");
  pose();cycle();require(layer.isCurrent(),"new authority did not recover");
  geometry_msgs::msg::PoseArray bad;bad.header.frame_id="map";bad.header.stamp=clock->now();poses->publish(bad);drain();local_grid();cycle();require(cost(1.,0.)==255&&!layer.isCurrent(),"invalid unified pose still allowed navigation");
  pose(3.,0.);local_grid();cycle(3.,0.);require(cost(0.,0.)==254,"old temporary near area did not restore Global");
  pose(0.,0.,-std::acos(-1.)/2);local_grid();cycle();require(cost(-1.,1.)==254,"map-odom rotation alignment failed");
  // Cache withdrawal must survive clock rollback and replay of the same source stamps.
  arx_base_mask::NearLocal cache;cache.initialize(node,"inflation","map");drain();
  auto future=clock->now().seconds()+10.;geometry_msgs::msg::PoseArray pa;pa.header.frame_id="map";pa.header.stamp=rclcpp::Time(int64_t(future*1e9));geometry_msgs::msg::Pose zero;zero.orientation.w=1.;pa.poses={zero,zero};auto fake=first;fake.header.stamp=pa.header.stamp;
  auto send=[&](){poses->publish(pa);local->publish(fake);drain();};send();cache.update(0,0,future+.2);require(cache.ready(),"fresh cache not ready");
  cache.update(0,0,future+.8);require(!cache.ready(),"expired cache ready");cache.update(0,0,future+.2);require(!cache.ready(),"expired cache revived on rollback");send();cache.update(0,0,future+.2);require(!cache.ready(),"replayed source revived cache");
  pa.header.stamp=rclcpp::Time(int64_t((future+.3)*1e9));fake.header.stamp=pa.header.stamp;send();cache.update(0,0,future+.4);require(cache.ready(),"new source stamp did not recover cache");
  std::cout<<"PASS native near-Local replacement, raw costs, real/far obstacles, rolling, stale/replay, pose invalidation and relocation\n";
 }catch(const std::exception&e){std::cerr<<"FAIL: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
