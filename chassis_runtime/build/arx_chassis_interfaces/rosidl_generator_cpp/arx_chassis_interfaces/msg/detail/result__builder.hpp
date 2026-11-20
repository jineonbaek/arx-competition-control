// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arx_chassis_interfaces:msg/Result.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/result.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__BUILDER_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arx_chassis_interfaces/msg/detail/result__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arx_chassis_interfaces
{

namespace msg
{

namespace builder
{

class Init_Result_state
{
public:
  explicit Init_Result_state(::arx_chassis_interfaces::msg::Result & msg)
  : msg_(msg)
  {}
  ::arx_chassis_interfaces::msg::Result state(::arx_chassis_interfaces::msg::Result::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Result msg_;
};

class Init_Result_output_seq
{
public:
  explicit Init_Result_output_seq(::arx_chassis_interfaces::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_state output_seq(::arx_chassis_interfaces::msg::Result::_output_seq_type arg)
  {
    msg_.output_seq = std::move(arg);
    return Init_Result_state(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Result msg_;
};

class Init_Result_reason
{
public:
  explicit Init_Result_reason(::arx_chassis_interfaces::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_output_seq reason(::arx_chassis_interfaces::msg::Result::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_Result_output_seq(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Result msg_;
};

class Init_Result_accepted
{
public:
  explicit Init_Result_accepted(::arx_chassis_interfaces::msg::Result & msg)
  : msg_(msg)
  {}
  Init_Result_reason accepted(::arx_chassis_interfaces::msg::Result::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Result_reason(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Result msg_;
};

class Init_Result_request_id
{
public:
  Init_Result_request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Result_accepted request_id(::arx_chassis_interfaces::msg::Result::_request_id_type arg)
  {
    msg_.request_id = std::move(arg);
    return Init_Result_accepted(msg_);
  }

private:
  ::arx_chassis_interfaces::msg::Result msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arx_chassis_interfaces::msg::Result>()
{
  return arx_chassis_interfaces::msg::builder::Init_Result_request_id();
}

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__BUILDER_HPP_
