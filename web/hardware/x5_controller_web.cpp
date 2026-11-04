#include "arm_limits.hpp"
#include "arx_x5_src/interfaces/InterfacesThread.hpp"
#include <rclcpp/rclcpp.hpp>
#include <arx5_arm_msg/msg/robot_status.hpp>
#include <arx5_arm_msg/msg/robot_cmd.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/file.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>
#include <chrono>
#include <thread>
using Clock=std::chrono::steady_clock;
constexpr auto ROOT="/home/arx/arx-competition-control";
class ArmCanFeedback {
  int fd_;
  std::array<Clock::time_point,7> received_{};
public:
  explicit ArmCanFeedback(const std::string& can) {
    fd_=socket(PF_CAN,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,CAN_RAW);
    sockaddr_can addr{};addr.can_family=AF_CAN;addr.can_ifindex=if_nametoindex(can.c_str());
    if(fd_<0 || !addr.can_ifindex || bind(fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0) throw std::runtime_error("Cannot monitor "+can);
  }
  ~ArmCanFeedback(){close(fd_);}
  bool fresh() {
    can_frame frame{};iovec io{&frame,sizeof(frame)};msghdr msg{};msg.msg_iov=&io;msg.msg_iovlen=1;
    for(int count=0;count<4096;++count) {
      msg.msg_flags=0;
      if(recvmsg(fd_,&msg,MSG_DONTWAIT)!=sizeof(frame)) break;
      if(frame.can_id&(CAN_EFF_FLAG|CAN_RTR_FLAG|CAN_ERR_FLAG)) continue;
      // Linux marks host-generated loopback with MSG_DONTROUTE; it is not motor feedback.
      int index=arm_feedback_index(frame.can_id,frame.can_dlc,frame.data[0],msg.msg_flags&MSG_DONTROUTE);
      if(index>=0) received_[index]=Clock::now();
    }
    auto now=Clock::now();
    return std::all_of(received_.begin(),received_.end(),[&](auto t){return t!=Clock::time_point{} && now-t<std::chrono::milliseconds(300);});
  }
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("x5_web_controller");
  std::string side=node->declare_parameter("side",std::string(""));
  if(side!="left" && side!="right") throw std::runtime_error("side must be left or right");
  const std::string can=side=="left"?"can1":"can3";
  std::ifstream file(std::string(ROOT)+"/web/config/control.json");
  auto config=nlohmann::json::parse(file).at("arms").at(side);
  ArmLimits limits{config.at("joint_limits_deg").get<decltype(ArmLimits::degrees)>(),config.at("gripper_limits_raw").get<decltype(ArmLimits::gripper)>(),config.at("max_delta_deg").get<double>()};
  int lock=open((std::string(ROOT)+"/web/hardware/runtime/"+can+".lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0600);
  if(lock<0 || flock(lock,LOCK_EX|LOCK_NB)!=0) throw std::runtime_error(can+" controller already running");
  ArmCanFeedback monitor(can);
  std::string urdf=std::string(ROOT)+"/repos/ARX_X5/ROS2/X5_ws/src/arx_x5_ros2/arx_x5_controller/x5_2025.urdf";
  auto arm=std::make_unique<arx::x5::InterfacesThread>(urdf,can,2);
  arm->arx_x(500,2000,10);
  arm->setArmStatus(2);
  arm->setHomePositions(std::vector<double>(6,0));
  auto pub=node->create_publisher<arx5_arm_msg::msg::RobotStatus>(config.at("state_topic").get<std::string>(),1);
  auto ready_pub=node->create_publisher<std_msgs::msg::Bool>("/web/"+side+"/ready",1);
  auto ack_pub=node->create_publisher<std_msgs::msg::String>(config.at("command_result_topic").get<std::string>(),10);
  double speed=config.at("joint_speed_deg_s").get<double>();
  if(!std::isfinite(speed) || speed<=0 || speed>5.7) throw std::runtime_error("Invalid joint speed");
  bool initialized=false,healthy=false,was_fresh=false,seen_feedback=false,fault_latched=false;
  bool joint_active=false;
  int current_mode=2;
  std::vector<double> reference(6,0),goal(6,0);
  auto last_step=Clock::now();
  auto hold=[&](){
    auto p=arm->getJointPositons();
    if(p.size()!=7 || !std::all_of(p.begin(),p.end(),[](double v){return std::isfinite(v);})) return false;
    joint_active=false;reference.assign(p.begin(),p.begin()+6);goal=reference;
    arm->setJointPositions(reference);arm->setCatch(p[6]);arm->setArmStatus(5);current_mode=5;return true;
  };
  auto sub=node->create_subscription<arx5_arm_msg::msg::RobotCmd>(config.at("command_topic").get<std::string>(),1,
    [&](arx5_arm_msg::msg::RobotCmd::ConstSharedPtr msg){
      auto separator=msg->header.frame_id.find('|');
      std::string kind=msg->header.frame_id.substr(0,separator);
      std::string id=separator==std::string::npos?"":msg->header.frame_id.substr(separator+1);
      std::vector<double> target(msg->joint_pos.begin(),msg->joint_pos.end());
      auto actual=arm->getJointPositons();
      std::string reason;
      if(!healthy || !monitor.fresh()) reason="Arm driver not ready or CAN feedback stale";
      else if(kind=="gripper") {
        if(msg->mode!=5 || !limits.valid_gripper(msg->gripper)) reason="Gripper target outside software limits";
      } else if(kind=="arm.joints") {
        if(msg->mode!=5 || !limits.valid(target,limits.gripper[1],actual,5)) reason="Joint target outside software limits";
      } else if(kind=="arm.mode") {
        if(msg->mode<0 || msg->mode>3) reason="Unsupported manual mode";
      } else reason="Unsupported command kind";
      if(reason.empty()) {
        if(kind=="gripper") {
          // Gripper commands preserve the arm goal; 0 is a valid close target.
          if(current_mode!=5) {joint_active=false;reference.assign(actual.begin(),actual.begin()+6);goal=reference;arm->setJointPositions(reference);}
          arm->setCatch(msg->gripper);arm->setArmStatus(5);current_mode=5;
        } else if(kind=="arm.joints") {
          reference.assign(actual.begin(),actual.begin()+6);goal=target;joint_active=true;last_step=Clock::now();
          arm->setJointPositions(reference);arm->setArmStatus(5);current_mode=5;
        } else {joint_active=false;arm->setArmStatus(msg->mode);current_mode=msg->mode;}
      }
      std_msgs::msg::String ack;
      ack.data=nlohmann::json{{"request_id",id},{"ok",reason.empty()},{"message",reason.empty()?"Driver accepted target":reason}}.dump();ack_pub->publish(ack);
      if(reason.empty()) RCLCPP_INFO(node->get_logger(),"Accepted %s %s",side.c_str(),kind.c_str());
      else RCLCPP_WARN(node->get_logger(),"Rejected %s %s: %s",side.c_str(),kind.c_str(),reason.c_str());
    });
  auto stop=node->create_service<std_srvs::srv::Trigger>("/web/"+side+"/stop",
    [&](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
      response->success=monitor.fresh() && hold();
      if(!response->success) arm->setArmStatus(2);
      response->message=response->success?"Holding measured arm and gripper positions":"Feedback unavailable; requested damping";
    });
  auto timer=node->create_wall_timer(std::chrono::milliseconds(10),[&](){
    bool fresh=monitor.fresh();
    auto errors=arm->getErrorCode();
    bool no_error=std::all_of(errors.begin(),errors.end(),[](int x){return x==0;});
    auto pos=arm->getJointPositons(),vel=arm->getJointVelocities(),cur=arm->getJointCurrent();
    bool valid=pos.size()==7 && vel.size()==7 && cur.size()==7 && std::all_of(pos.begin(),pos.end(),[](double x){return std::isfinite(x);});
    if(fresh && valid && no_error && !initialized && !fault_latched){initialized=hold();RCLCPP_INFO(node->get_logger(),"%s measured-position hold initialized",side.c_str());}
    healthy=fresh && valid && no_error && initialized && !fault_latched;
    if((!fresh && was_fresh) || (!no_error && seen_feedback)) {arm->setArmStatus(2);initialized=false;fault_latched=true;joint_active=false;}
    if(fresh!=was_fresh) RCLCPP_INFO(node->get_logger(),"%s CAN feedback %s",side.c_str(),fresh?"fresh":"stale");
    was_fresh=fresh;seen_feedback|=fresh;
    std_msgs::msg::Bool ready;ready.data=healthy;ready_pub->publish(ready);
    if(!fresh || !valid) return;
    if(healthy && joint_active) {
      auto now=Clock::now();double dt=std::min(.02,std::chrono::duration<double>(now-last_step).count());last_step=now;
      reference=limits.advance(reference,goal,pos,speed,dt);arm->setJointPositions(reference);
      if(reference==goal) joint_active=false;
    }
    auto xyz=arx::solve::Isometry2Xyzrpy(arm->getEndPose());
    if(xyz.size()!=6 || !std::all_of(xyz.begin(),xyz.end(),[](double x){return std::isfinite(x);})) return;
    arx5_arm_msg::msg::RobotStatus msg;msg.header.stamp=node->now();
    std::copy(pos.begin(),pos.end(),msg.joint_pos.begin());std::copy(vel.begin(),vel.end(),msg.joint_vel.begin());
    std::copy(cur.begin(),cur.end(),msg.joint_cur.begin());std::copy(xyz.begin(),xyz.end(),msg.end_pos.begin());pub->publish(msg);
  });
  RCLCPP_INFO(node->get_logger(),"%s X5 2025 dedicated web driver started on %s",side.c_str(),can.c_str());
  rclcpp::spin(node);
  arm->setArmStatus(2);std::this_thread::sleep_for(std::chrono::milliseconds(30));arm.reset();
  close(lock);return 0;
}
