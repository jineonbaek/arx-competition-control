// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arx_chassis_interfaces:msg/State.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/state.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__BUILDER_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arx_chassis_interfaces/msg/detail/state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arx_chassis_interfaces
{

namespace msg
{

namespace builder
{

class Init_State_reason
{
public:
  explicit Init_State_reason(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  ::arx_chassis_interfaces::msg::State reason(::arx_chassis_interfaces::msg::State::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_applied_seq
{
public:
  explicit Init_State_applied_seq(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_reason applied_seq(::arx_chassis_interfaces::msg::State::_applied_seq_type arg)
  {
    msg_.applied_seq = std::move(arg);
    return Init_State_reason(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_applied_sdk
{
public:
  explicit Init_State_applied_sdk(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_applied_seq applied_sdk(::arx_chassis_interfaces::msg::State::_applied_sdk_type arg)
  {
    msg_.applied_sdk = std::move(arg);
    return Init_State_applied_seq(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_maintenance
{
public:
  explicit Init_State_maintenance(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_applied_sdk maintenance(::arx_chassis_interfaces::msg::State::_maintenance_type arg)
  {
    msg_.maintenance = std::move(arg);
    return Init_State_applied_sdk(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_sdk
{
public:
  explicit Init_State_sdk(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_maintenance sdk(::arx_chassis_interfaces::msg::State::_sdk_type arg)
  {
    msg_.sdk = std::move(arg);
    return Init_State_maintenance(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_expires_at
{
public:
  explicit Init_State_expires_at(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_sdk expires_at(::arx_chassis_interfaces::msg::State::_expires_at_type arg)
  {
    msg_.expires_at = std::move(arg);
    return Init_State_sdk(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_output_seq
{
public:
  explicit Init_State_output_seq(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_expires_at output_seq(::arx_chassis_interfaces::msg::State::_output_seq_type arg)
  {
    msg_.output_seq = std::move(arg);
    return Init_State_expires_at(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_generation
{
public:
  explicit Init_State_generation(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_output_seq generation(::arx_chassis_interfaces::msg::State::_generation_type arg)
  {
    msg_.generation = std::move(arg);
    return Init_State_output_seq(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_task_id
{
public:
  explicit Init_State_task_id(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_generation task_id(::arx_chassis_interfaces::msg::State::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_State_generation(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_source_id
{
public:
  explicit Init_State_source_id(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_task_id source_id(::arx_chassis_interfaces::msg::State::_source_id_type arg)
  {
    msg_.source_id = std::move(arg);
    return Init_State_task_id(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_client_id
{
public:
  explicit Init_State_client_id(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_source_id client_id(::arx_chassis_interfaces::msg::State::_client_id_type arg)
  {
    msg_.client_id = std::move(arg);
    return Init_State_source_id(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_active
{
public:
  explicit Init_State_active(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_client_id active(::arx_chassis_interfaces::msg::State::_active_type arg)
  {
    msg_.active = std::move(arg);
    return Init_State_client_id(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_driver_instance
{
public:
  explicit Init_State_driver_instance(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_active driver_instance(::arx_chassis_interfaces::msg::State::_driver_instance_type arg)
  {
    msg_.driver_instance = std::move(arg);
    return Init_State_active(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_ready
{
public:
  explicit Init_State_ready(::arx_chassis_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_driver_instance ready(::arx_chassis_interfaces::msg::State::_ready_type arg)
  {
    msg_.ready = std::move(arg);
    return Init_State_driver_instance(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

class Init_State_scheduler_instance
{
public:
  Init_State_scheduler_instance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_State_ready scheduler_instance(::arx_chassis_interfaces::msg::State::_scheduler_instance_type arg)
  {
    msg_.scheduler_instance = std::move(arg);
    return Init_State_ready(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::State msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arx_chassis_interfaces::msg::State>()
{
  return arx_chassis_interfaces::msg::builder::Init_State_scheduler_instance();
}

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__BUILDER_HPP_
