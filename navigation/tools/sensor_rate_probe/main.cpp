#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
using Clock=std::chrono::steady_clock;
struct Stats{uint64_t n=0,bytes=0,points=0,missed=0,backward=0;double first=0,last=0,first_stamp=0,last_stamp=0,max_gap=0,expected=0,age_sum=0,max_age=-1e12;};
int main(int argc,char**argv){
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("sensor_rate_probe");
 double duration=node->declare_parameter("duration",20.0);auto output=node->declare_parameter("output",std::string("/tmp/sensor-rates.csv"));
 auto reliability=node->declare_parameter("reliability",std::string("best_effort"));
 auto mode=node->declare_parameter("topics",std::string("all"));
 auto qos=rclcpp::QoS(rclcpp::KeepLast(100));if(reliability=="reliable")qos.reliable();else qos.best_effort();
 std::map<std::string,Stats> stats;bool collect=false;
 auto receive=[&](const std::string&t,const builtin_interfaces::msg::Time&stamp,size_t bytes,size_t points){
  if(!collect)return;auto&s=stats[t];double now=std::chrono::duration<double>(Clock::now().time_since_epoch()).count();double ts=stamp.sec+stamp.nanosec*1e-9;
  if(s.n==0){s.first=now;s.first_stamp=ts;}else{double gap=ts-s.last_stamp;if(gap<0)s.backward++;if(gap>0){s.max_gap=std::max(s.max_gap,gap);if(s.expected>0 && gap>1.8/s.expected)s.missed+=std::max(0LL,std::llround(gap*s.expected)-1);}}
  double age=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()-ts;s.age_sum+=age;s.max_age=std::max(s.max_age,age);s.n++;s.last=now;s.last_stamp=ts;s.bytes+=bytes;s.points+=points;
 };
 std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
 if(mode=="all"||mode=="camera")for(auto t:{std::string("/sensors/d435/color/image_raw"),std::string("/sensors/d435/depth/image_rect_raw")})
  subs.push_back(node->create_subscription<sensor_msgs::msg::Image>(t,qos,[&,t](sensor_msgs::msg::Image::ConstSharedPtr m){receive(t,m->header.stamp,m->data.size(),0);}));
 if(mode=="all"||mode=="lidar")for(auto suffix:{std::string("192_168_1_3"),std::string("192_168_1_164")}){
  auto t="/livox/lidar_"+suffix;stats[t].expected=10;
  subs.push_back(node->create_subscription<livox_ros_driver2::msg::CustomMsg>(t,qos,[&,t](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr m){receive(t,m->header.stamp,m->point_num*19,m->point_num);}));
  auto imu="/livox/imu_"+suffix;stats[imu].expected=200;
  subs.push_back(node->create_subscription<sensor_msgs::msg::Imu>(imu,qos,[&,imu](sensor_msgs::msg::Imu::ConstSharedPtr m){receive(imu,m->header.stamp,0,0);}));
 }
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
 auto begin=Clock::now();while(rclcpp::ok()&&std::chrono::duration<double>(Clock::now()-begin).count()<5){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 collect=true;begin=Clock::now();while(rclcpp::ok()&&std::chrono::duration<double>(Clock::now()-begin).count()<duration){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 std::ofstream out(output);out<<"topic,messages,wall_hz,stamp_hz,MB_s,points_s,max_stamp_gap_ms,estimated_missing_200hz_or_10hz,backwards,mean_header_age_ms,max_header_age_ms\n"<<std::fixed<<std::setprecision(3);
 for(auto&[topic,s]:stats){double dt=s.last-s.first;double st=s.last_stamp-s.first_stamp;double hz=s.n>1&&dt>0?(s.n-1)/dt:0;double stamp_hz=s.n>1&&st>0?(s.n-1)/st:0;double window=dt+(s.expected>0?1/s.expected:(hz>0?1/hz:0));out<<topic<<","<<s.n<<","<<hz<<","<<stamp_hz<<","<<(window>0?s.bytes/window/1e6:0)<<","<<(window>0?s.points/window:0)<<","<<s.max_gap*1000<<","<<s.missed<<","<<s.backward<<","<<(s.n?s.age_sum/s.n*1000:0)<<","<<(s.n?s.max_age*1000:0)<<"\n";}
 out.close();executor.remove_node(node);rclcpp::shutdown();return 0;
}
