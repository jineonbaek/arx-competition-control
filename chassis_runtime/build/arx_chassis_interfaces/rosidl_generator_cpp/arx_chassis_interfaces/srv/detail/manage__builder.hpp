// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arx_chassis_interfaces:srv/Manage.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/srv/manage.hpp"


#ifndef ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__BUILDER_HPP_
#define ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arx_chassis_interfaces/srv/detail/manage__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arx_chassis_interfaces
{

namespace srv
{

namespace builder
{

class Init_Manage_Request_output_seq
{
public:
  explicit Init_Manage_Request_output_seq(::arx_chassis_interfaces::srv::Manage_Request & msg)
  : msg_(msg)
  {}
  ::arx_chassis_interfaces::srv::Manage_Request output_seq(::arx_chassis_interfaces::srv::Manage_Request::_output_seq_type arg)
  {
    msg_.output_seq = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Request msg_;
};

class Init_Manage_Request_resume_key
{
public:
  explicit Init_Manage_Request_resume_key(::arx_chassis_interfaces::srv::Manage_Request & msg)
  : msg_(msg)
  {}
  Init_Manage_Request_output_seq resume_key(::arx_chassis_interfaces::srv::Manage_Request::_resume_key_type arg)
  {
    msg_.resume_key = std::move(arg);
    return Init_Manage_Request_output_seq(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Request msg_;
};

class Init_Manage_Request_profile
{
public:
  explicit Init_Manage_Request_profile(::arx_chassis_interfaces::srv::Manage_Request & msg)
  : msg_(msg)
  {}
  Init_Manage_Request_resume_key profile(::arx_chassis_interfaces::srv::Manage_Request::_profile_type arg)
  {
    msg_.profile = std::move(arg);
    return Init_Manage_Request_resume_key(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Request msg_;
};

class Init_Manage_Request_client_id
{
public:
  explicit Init_Manage_Request_client_id(::arx_chassis_interfaces::srv::Manage_Request & msg)
  : msg_(msg)
  {}
  Init_Manage_Request_profile client_id(::arx_chassis_interfaces::srv::Manage_Request::_client_id_type arg)
  {
    msg_.client_id = std::move(arg);
    return Init_Manage_Request_profile(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Request msg_;
};

class Init_Manage_Request_scheduler_instance
{
public:
  explicit Init_Manage_Request_scheduler_instance(::arx_chassis_interfaces::srv::Manage_Request & msg)
  : msg_(msg)
  {}
  Init_Manage_Request_client_id scheduler_instance(::arx_chassis_interfaces::srv::Manage_Request::_scheduler_instance_type arg)
  {
    msg_.scheduler_instance = std::move(arg);
    return Init_Manage_Request_client_id(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Request msg_;
};

class Init_Manage_Request_operation
{
public:
  Init_Manage_Request_operation()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Manage_Request_scheduler_instance operation(::arx_chassis_interfaces::srv::Manage_Request::_operation_type arg)
  {
    msg_.operation = std::move(arg);
    return Init_Manage_Request_scheduler_instance(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::arx_chassis_interfaces::srv::Manage_Request>()
{
  return arx_chassis_interfaces::srv::builder::Init_Manage_Request_operation();
}

}  // namespace arx_chassis_interfaces


namespace arx_chassis_interfaces
{

namespace srv
{

namespace builder
{

class Init_Manage_Response_state
{
public:
  explicit Init_Manage_Response_state(::arx_chassis_interfaces::srv::Manage_Response & msg)
  : msg_(msg)
  {}
  ::arx_chassis_interfaces::srv::Manage_Response state(::arx_chassis_interfaces::srv::Manage_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Response msg_;
};

class Init_Manage_Response_client_id
{
public:
  explicit Init_Manage_Response_client_id(::arx_chassis_interfaces::srv::Manage_Response & msg)
  : msg_(msg)
  {}
  Init_Manage_Response_state client_id(::arx_chassis_interfaces::srv::Manage_Response::_client_id_type arg)
  {
    msg_.client_id = std::move(arg);
    return Init_Manage_Response_state(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Response msg_;
};

class Init_Manage_Response_reason
{
public:
  explicit Init_Manage_Response_reason(::arx_chassis_interfaces::srv::Manage_Response & msg)
  : msg_(msg)
  {}
  Init_Manage_Response_client_id reason(::arx_chassis_interfaces::srv::Manage_Response::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_Manage_Response_client_id(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Response msg_;
};

class Init_Manage_Response_accepted
{
public:
  Init_Manage_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Manage_Response_reason accepted(::arx_chassis_interfaces::srv::Manage_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Manage_Response_reason(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::arx_chassis_interfaces::srv::Manage_Response>()
{
  return arx_chassis_interfaces::srv::builder::Init_Manage_Response_accepted();
}

}  // namespace arx_chassis_interfaces


namespace arx_chassis_interfaces
{

namespace srv
{

namespace builder
{

class Init_Manage_Event_response
{
public:
  explicit Init_Manage_Event_response(::arx_chassis_interfaces::srv::Manage_Event & msg)
  : msg_(msg)
  {}
  ::arx_chassis_interfaces::srv::Manage_Event response(::arx_chassis_interfaces::srv::Manage_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Event msg_;
};

class Init_Manage_Event_request
{
public:
  explicit Init_Manage_Event_request(::arx_chassis_interfaces::srv::Manage_Event & msg)
  : msg_(msg)
  {}
  Init_Manage_Event_response request(::arx_chassis_interfaces::srv::Manage_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Manage_Event_response(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Event msg_;
};

class Init_Manage_Event_info
{
public:
  Init_Manage_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Manage_Event_request info(::arx_chassis_interfaces::srv::Manage_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Manage_Event_request(msg_);
  }

private:
  ::arx_chassis_interfaces::srv::Manage_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::arx_chassis_interfaces::srv::Manage_Event>()
{
  return arx_chassis_interfaces::srv::builder::Init_Manage_Event_info();
}

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__BUILDER_HPP_
