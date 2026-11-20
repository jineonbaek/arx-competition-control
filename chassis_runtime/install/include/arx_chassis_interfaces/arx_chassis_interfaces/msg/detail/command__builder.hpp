// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arx_chassis_interfaces:msg/Command.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/command.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__BUILDER_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arx_chassis_interfaces/msg/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arx_chassis_interfaces
{

namespace msg
{

namespace builder
{

class Init_Command_sdk
{
public:
  explicit Init_Command_sdk(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  ::arx_chassis_interfaces::msg::Command sdk(::arx_chassis_interfaces::msg::Command::_sdk_type arg)
  {
    msg_.sdk = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_ttl_s
{
public:
  explicit Init_Command_ttl_s(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_sdk ttl_s(::arx_chassis_interfaces::msg::Command::_ttl_s_type arg)
  {
    msg_.ttl_s = std::move(arg);
    return Init_Command_sdk(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_issued_at
{
public:
  explicit Init_Command_issued_at(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_ttl_s issued_at(::arx_chassis_interfaces::msg::Command::_issued_at_type arg)
  {
    msg_.issued_at = std::move(arg);
    return Init_Command_ttl_s(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_seq
{
public:
  explicit Init_Command_seq(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_issued_at seq(::arx_chassis_interfaces::msg::Command::_seq_type arg)
  {
    msg_.seq = std::move(arg);
    return Init_Command_issued_at(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_generation
{
public:
  explicit Init_Command_generation(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_seq generation(::arx_chassis_interfaces::msg::Command::_generation_type arg)
  {
    msg_.generation = std::move(arg);
    return Init_Command_seq(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_task_id
{
public:
  explicit Init_Command_task_id(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_generation task_id(::arx_chassis_interfaces::msg::Command::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_Command_generation(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_source_id
{
public:
  explicit Init_Command_source_id(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_task_id source_id(::arx_chassis_interfaces::msg::Command::_source_id_type arg)
  {
    msg_.source_id = std::move(arg);
    return Init_Command_task_id(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_kind
{
public:
  explicit Init_Command_kind(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_source_id kind(::arx_chassis_interfaces::msg::Command::_kind_type arg)
  {
    msg_.kind = std::move(arg);
    return Init_Command_source_id(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_operation
{
public:
  explicit Init_Command_operation(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_kind operation(::arx_chassis_interfaces::msg::Command::_operation_type arg)
  {
    msg_.operation = std::move(arg);
    return Init_Command_kind(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_client_id
{
public:
  explicit Init_Command_client_id(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_operation client_id(::arx_chassis_interfaces::msg::Command::_client_id_type arg)
  {
    msg_.client_id = std::move(arg);
    return Init_Command_operation(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_scheduler_instance
{
public:
  explicit Init_Command_scheduler_instance(::arx_chassis_interfaces::msg::Command & msg)
  : msg_(msg)
  {}
  Init_Command_client_id scheduler_instance(::arx_chassis_interfaces::msg::Command::_scheduler_instance_type arg)
  {
    msg_.scheduler_instance = std::move(arg);
    return Init_Command_client_id(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

class Init_Command_request_id
{
public:
  Init_Command_request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Command_scheduler_instance request_id(::arx_chassis_interfaces::msg::Command::_request_id_type arg)
  {
    msg_.request_id = std::move(arg);
    return Init_Command_scheduler_instance(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Command msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arx_chassis_interfaces::msg::Command>()
{
  return arx_chassis_interfaces::msg::builder::Init_Command_request_id();
}

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__BUILDER_HPP_
