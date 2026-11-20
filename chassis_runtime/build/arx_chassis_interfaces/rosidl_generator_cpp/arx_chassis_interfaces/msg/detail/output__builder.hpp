// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arx_chassis_interfaces:msg/Output.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/output.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__BUILDER_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arx_chassis_interfaces/msg/detail/output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arx_chassis_interfaces
{

namespace msg
{

namespace builder
{

class Init_Output_source_id
{
public:
  explicit Init_Output_source_id(::arx_chassis_interfaces::msg::Output & msg)
  : msg_(msg)
  {}
  ::arx_chassis_interfaces::msg::Output source_id(::arx_chassis_interfaces::msg::Output::_source_id_type arg)
  {
    msg_.source_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Output msg_;
};

class Init_Output_reason
{
public:
  explicit Init_Output_reason(::arx_chassis_interfaces::msg::Output & msg)
  : msg_(msg)
  {}
  Init_Output_source_id reason(::arx_chassis_interfaces::msg::Output::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_Output_source_id(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Output msg_;
};

class Init_Output_sdk
{
public:
  explicit Init_Output_sdk(::arx_chassis_interfaces::msg::Output & msg)
  : msg_(msg)
  {}
  Init_Output_reason sdk(::arx_chassis_interfaces::msg::Output::_sdk_type arg)
  {
    msg_.sdk = std::move(arg);
    return Init_Output_reason(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Output msg_;
};

class Init_Output_ttl_s
{
public:
  explicit Init_Output_ttl_s(::arx_chassis_interfaces::msg::Output & msg)
  : msg_(msg)
  {}
  Init_Output_sdk ttl_s(::arx_chassis_interfaces::msg::Output::_ttl_s_type arg)
  {
    msg_.ttl_s = std::move(arg);
    return Init_Output_sdk(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Output msg_;
};

class Init_Output_issued_at
{
public:
  explicit Init_Output_issued_at(::arx_chassis_interfaces::msg::Output & msg)
  : msg_(msg)
  {}
  Init_Output_ttl_s issued_at(::arx_chassis_interfaces::msg::Output::_issued_at_type arg)
  {
    msg_.issued_at = std::move(arg);
    return Init_Output_ttl_s(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Output msg_;
};

class Init_Output_output_seq
{
public:
  explicit Init_Output_output_seq(::arx_chassis_interfaces::msg::Output & msg)
  : msg_(msg)
  {}
  Init_Output_issued_at output_seq(::arx_chassis_interfaces::msg::Output::_output_seq_type arg)
  {
    msg_.output_seq = std::move(arg);
    return Init_Output_issued_at(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Output msg_;
};

class Init_Output_driver_instance
{
public:
  explicit Init_Output_driver_instance(::arx_chassis_interfaces::msg::Output & msg)
  : msg_(msg)
  {}
  Init_Output_output_seq driver_instance(::arx_chassis_interfaces::msg::Output::_driver_instance_type arg)
  {
    msg_.driver_instance = std::move(arg);
    return Init_Output_output_seq(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Output msg_;
};

class Init_Output_scheduler_instance
{
public:
  Init_Output_scheduler_instance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Output_driver_instance scheduler_instance(::arx_chassis_interfaces::msg::Output::_scheduler_instance_type arg)
  {
    msg_.scheduler_instance = std::move(arg);
    return Init_Output_driver_instance(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Output msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arx_chassis_interfaces::msg::Output>()
{
  return arx_chassis_interfaces::msg::builder::Init_Output_scheduler_instance();
}

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__BUILDER_HPP_
