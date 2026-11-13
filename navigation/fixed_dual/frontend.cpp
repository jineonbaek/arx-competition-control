#include "windows.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
using Cloud=livox_ros_driver2::msg::CustomMsg;
using Json=nlohmann::json;
class Frontend:public rclcpp::Node{
 Windows windows;std::string root,run,fault;
 rclcpp::Publisher<Cloud>::SharedPtr pub;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
 std::array<rclcpp::Subscription<Cloud>::SharedPtr,2> subs;
 rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu;
 rclcpp::TimerBase::SharedPtr timer;
 int64_t imu_stamp=0,last_output=0;size_t frames=0;std::array<size_t,2> counts{};
 void fail(const std::string &reason){fault=reason;RCLCPP_ERROR(get_logger(),"%s",reason.c_str());}
 public:
 Frontend(std::string r,std::string o):Node("fixed_dual_frontend"),root(r),run(o){
   pub=create_publisher<Cloud>("/fixed_dual/lidar",10);status=create_publisher<std_msgs::msg::String>("/fixed_dual/status",2);
   for(int s=0;s<2;s++)subs[s]=create_subscription<Cloud>(std::string("/fixed_dual/")+(s==0?"bottom":"head")+"/filtered",rclcpp::QoS(20).best_effort(),[this,s](Cloud::ConstSharedPtr m){cloud(s,*m);});
   imu=create_subscription<sensor_msgs::msg::Imu>("/single/imu",rclcpp::QoS(1000).best_effort(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
     auto t=rclcpp::Time(m->header.stamp).nanoseconds();if(imu_stamp&&t<=imu_stamp)fail("IMU clock reversed; restart stationary");imu_stamp=t;drain();});
   timer=create_wall_timer(std::chrono::milliseconds(200),[this]{
     try{std::ifstream file(root+"/config/lidar_streams.json");Json cfg;file>>cfg;windows.set_enabled({cfg.at("bottom").get<bool>(),cfg.at("head").get<bool>()});}
     catch(const std::exception &e){fail(std::string("input switch read: ")+e.what());}
     auto t=now().nanoseconds();std::string state=fault.empty()?"ready":fault;
     for(int s=0;s<2;s++)if(windows.enabled[s]&&t-windows.latest[s]>500000000)state="waiting for enabled source "+std::to_string(s);
     if(t-imu_stamp>150000000)state="waiting for bottom IMU";
     Json report={{"mode",state},{"fault",fault},{"frames",frames},{"bottom_points",counts[0]},{"rear_points",counts[1]},{"last_output_ns",last_output},{"enabled",windows.enabled},{"commands_sent",0}};
     std_msgs::msg::String m;m.data=report.dump();status->publish(m);std::ofstream(run+"/fixed-fusion.json")<<report.dump(2);drain();
   });
 }
 void cloud(int s,const Cloud &m){
   if(!fault.empty()||!windows.enabled[s])return;
   int64_t t=rclcpp::Time(m.header.stamp).nanoseconds(),age=now().nanoseconds()-t;
   if(age>600000000||age< -25000000)return;
   if(std::llabs(t-int64_t(m.timebase))>1000000||m.point_num!=m.points.size()){fail("malformed point clock");return;}
   std::vector<Sample> points;points.reserve(m.points.size());
   for(auto &p:m.points){
     if(p.offset_time>150000000){fail("point offset outside scan");return;}
     if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
     if(std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z)<.2)continue;
     Sample a{int64_t(m.timebase)+p.offset_time,s,p.x,p.y,p.z,p.reflectivity,p.tag,p.line};
     // Rear optical -> front optical, not IMU; FAST-LIO applies its existing front->IMU extrinsic.
     if(s==1){a.x=-p.y-.46853f;a.y=p.x+.000003f;a.z=p.z-.002f;}
     points.push_back(a);
   }
   try{windows.add(s,t,std::move(points));drain();}catch(const std::exception &e){fail(e.what());}
 }
 void drain(){
   if(!fault.empty()||!imu_stamp||now().nanoseconds()-imu_stamp>150000000)return;
   while(true){
     auto window=windows.pop();if(!window)return;
     if(window->start<=last_output){fail("fused clock regression");return;}
     if(now().nanoseconds()-window->end>500000000)continue;
     std::array<size_t,2> n{};for(auto &p:window->points)n[p.source]++;
     for(int s=0;s<2;s++)if(windows.enabled[s]&&n[s]<30){fail("enabled source has too few points");return;}
     Cloud out;out.header.frame_id="bottom_livox";out.header.stamp=rclcpp::Time(window->start);out.timebase=window->start;out.lidar_id=0;
     for(auto &p:window->points){livox_ros_driver2::msg::CustomPoint q;q.x=p.x;q.y=p.y;q.z=p.z;q.reflectivity=p.reflectivity;q.tag=p.tag;q.line=p.line;q.offset_time=p.time-window->start;out.points.push_back(q);}
     out.point_num=out.points.size();pub->publish(out);frames++;last_output=window->start;for(int s=0;s<2;s++)counts[s]+=n[s];
   }
 }
};
int main(int argc,char **argv){rclcpp::init(argc,argv);if(argc<3)return 2;auto n=std::make_shared<Frontend>(argv[1],argv[2]);rclcpp::spin(n);rclcpp::shutdown();}
