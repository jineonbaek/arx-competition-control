//
// Created by nuc-ros2 on 24-12-6.
//

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <arx_lift_src/lift_head_control_loop.h>
#include <arm_control/msg/pos_cmd.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <csignal>
#include <cmath>
#include <stdexcept>
#include "slow_lift_config.hpp"
#include "chassis_motion.hpp"
#include "chassis_output.hpp"
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

class WebLiftControl : public LiftHeadControlLoop {
 public:
  using LiftHeadControlLoop::LiftHeadControlLoop;
  bool calibrated() const { return lift_is_calibrated_; }
  double reference() const { return lift_pos_current_des_; }
};
std::shared_ptr<WebLiftControl> control_loop;
void signalHandler(int signum) {
  control_loop.reset();
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  std::signal(SIGHUP, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGINT, signalHandler);

  const int lock_fd = open("/home/arx/arx-competition-control/web/hardware/runtime/can5.lock", O_CREAT | O_RDWR, 0600);
  if (lock_fd < 0 || flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
    throw std::runtime_error("Web CAN5 controller is already running");
  }
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lift_controller");
  int robot_type = node->declare_parameter("robot_type", 0);
  const double fixed_body_height =
      node->declare_parameter("fixed_body_height", 0.0);
  if (!valid_lift_target(fixed_body_height) || fixed_body_height != 0.0 || robot_type != 0) {
    throw std::invalid_argument("Height target must be finite and within 0..18 SDK radians");
  }
  auto target_validation_handle = node->add_on_set_parameters_callback(
    [](const std::vector<rclcpp::Parameter>& parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto& parameter : parameters) {
        if (parameter.get_name() == "fixed_body_height" &&
            (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
             !valid_lift_target(parameter.as_double()) || !control_loop || !control_loop->calibrated())) {
          result.successful = false;
          result.reason = "Target must be a double within 0..18 SDK radians";
          break;
        }
      }
      return result;
    });
  control_loop = std::make_shared<WebLiftControl>("can5", static_cast<LiftHeadControlLoop::RobotType>(robot_type));
  apply_slow_lift_config(control_loop->config_);
  RCLCPP_INFO(node->get_logger(), "LOW SPEED: target ramp 0.5rad/s; homing 0.5rad/s; target %.4frad", fixed_body_height);
  int running_state = 2;
  double lift_height = 0;
  auto pub = node->create_publisher<arm_control::msg::PosCmd>("/body_information", 1);
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/arx_imu", 1);
  auto ready_pub = node->create_publisher<std_msgs::msg::Bool>("/web/body/calibrated", 1);
  auto last_command = std::chrono::steady_clock::time_point{};
  ChassisMotion chassis_motion(2.0, 2.0);
  const std::string driver_instance=std::to_string(getpid())+"-"+
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  ChassisOutputGate output_gate(driver_instance);
  auto output_deadline=std::chrono::steady_clock::time_point{};
  auto output_status=node->create_publisher<std_msgs::msg::String>("/web/chassis/output_status",1);
  // Sole chassis input. Legacy web/nav/gamepad direct channels are intentionally
  // absent: they would bypass Controller validation and maintenance exclusivity.
  auto chassis_sub=node->create_subscription<std_msgs::msg::String>(
      "/web/chassis/ordered_output",10,[&](const std_msgs::msg::String &msg){
        try {
          if(msg.data.size()>4096)return;
          const auto j=nlohmann::json::parse(msg.data);
          if(!j.at("output_seq").is_number_unsigned() || !j.at("sdk").is_array() ||
             j.at("sdk").size()!=3)return;
          for(const auto &v:j.at("sdk"))if(!v.is_number())return;
          if(!j.at("issued_at").is_number() || !j.at("ttl_s").is_number())return;
          ChassisOutput o{j.at("driver_instance").get<std::string>(),
              j.at("output_seq").get<uint64_t>(),j.at("issued_at").get<double>(),
              j.at("ttl_s").get<double>(),j.at("sdk").get<std::array<double,3>>()};
          const auto now=std::chrono::steady_clock::now();
          const double wall=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
          if(!output_gate.accept(o,wall,control_loop->calibrated()))return;
          const double stamp=std::chrono::duration<double>(now.time_since_epoch()).count();
          last_command=now;
          output_deadline=now+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(std::max(0.,o.issued+o.ttl-wall)));
          const int mode=o.sdk==std::array<double,3>{0.,0.,0.}?2:1;
          chassis_motion.request(o.sdk,mode,stamp);
        }catch(const std::exception &e){
          RCLCPP_WARN_THROTTLE(node->get_logger(),*node->get_clock(),2000,
                              "Rejected chassis output: %s",e.what());
        }
      });
  unsigned status_tick=0;
  auto stop_lift_service = node->create_service<std_srvs::srv::Trigger>(
      "/web/body/stop_lift", [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const double reference = control_loop->reference();
        if (!control_loop->calibrated() || !valid_lift_target(reference)) {
          response->success = false;
          response->message = "Lift is not calibrated or reference is invalid";
          return;
        }
        auto result = node->set_parameter(rclcpp::Parameter("fixed_body_height", reference));
        response->success = result.successful;
        response->message = "Remaining lift target cancelled; holding current control reference";
      });
  rclcpp::Rate loop_rate(400);
  while (rclcpp::ok()) {
    // Lift target is independent of chassis messages; no head/waist/VR/joy inputs.
    control_loop->setHeight(node->get_parameter("fixed_body_height").as_double());
    const auto now = std::chrono::steady_clock::now();
    chassis_motion.step(std::chrono::duration<double>(now.time_since_epoch()).count(),
                        now>=output_deadline ? WEB_CHASSIS_TIMEOUT : std::chrono::duration<double>(now - last_command).count(),
                        control_loop->calibrated());
    const auto &velocity = chassis_motion.velocity();
    control_loop->setChassisCmd(velocity[0], velocity[1], velocity[2], chassis_motion.mode());
    control_loop->setWheelVel(0, 0, 0, 0);
    control_loop->loop();
    std_msgs::msg::Bool ready_msg;
    ready_msg.data = control_loop->calibrated();
    ready_pub->publish(ready_msg);
    if(++status_tick%20==0){
      std_msgs::msg::String state;
      state.data=nlohmann::json{{"driver_instance",driver_instance},
          {"output_seq",output_gate.sequence()},{"ordered_output",true},
          {"calibrated",control_loop->calibrated()}}.dump();
      output_status->publish(state);
    }
    arm_control::msg::PosCmd msg;
    msg.head_yaw = control_loop->getHeadYaw();
    msg.head_pit = control_loop->getHeadPitch();
    msg.height = control_loop->getHeight();
    msg.temp_float_data[0] = control_loop->getWaistPos();
    double wheel_vel[4];
    control_loop->getWheelVel(wheel_vel);
    for(int i = 0;i < 4; i++)
      msg.temp_float_data[i + 1] = wheel_vel[i];
    pub->publish(msg);
    sensor_msgs::msg::Imu imu_msg;
    double orientation[3],angular_vel[3],accel[3];
    control_loop->getOrientation(orientation);
    control_loop->getAngularVel(angular_vel);
    control_loop->getAccel(accel);
    imu_msg.header.stamp = rclcpp::Clock().now();
    tf2::Quaternion q;
    q.setRPY(orientation[0],orientation[1],orientation[2]);
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    imu_msg.linear_acceleration.x = accel[0];
    imu_msg.linear_acceleration.y = accel[1];
    imu_msg.linear_acceleration.z = accel[2];
    imu_msg.angular_velocity.x = angular_vel[0];
    imu_msg.angular_velocity.y = angular_vel[1];
    imu_msg.angular_velocity.z = angular_vel[2];
    imu_pub->publish(imu_msg);
    spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
