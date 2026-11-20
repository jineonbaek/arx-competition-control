// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arx_chassis_interfaces:srv/Manage.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/srv/manage.hpp"


#ifndef ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__TRAITS_HPP_
#define ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arx_chassis_interfaces/srv/detail/manage__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace arx_chassis_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Manage_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: operation
  {
    out << "operation: ";
    rosidl_generator_traits::value_to_yaml(msg.operation, out);
    out << ", ";
  }

  // member: scheduler_instance
  {
    out << "scheduler_instance: ";
    rosidl_generator_traits::value_to_yaml(msg.scheduler_instance, out);
    out << ", ";
  }

  // member: client_id
  {
    out << "client_id: ";
    rosidl_generator_traits::value_to_yaml(msg.client_id, out);
    out << ", ";
  }

  // member: profile
  {
    out << "profile: ";
    rosidl_generator_traits::value_to_yaml(msg.profile, out);
    out << ", ";
  }

  // member: resume_key
  {
    out << "resume_key: ";
    rosidl_generator_traits::value_to_yaml(msg.resume_key, out);
    out << ", ";
  }

  // member: output_seq
  {
    out << "output_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.output_seq, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Manage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: operation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation: ";
    rosidl_generator_traits::value_to_yaml(msg.operation, out);
    out << "\n";
  }

  // member: scheduler_instance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scheduler_instance: ";
    rosidl_generator_traits::value_to_yaml(msg.scheduler_instance, out);
    out << "\n";
  }

  // member: client_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "client_id: ";
    rosidl_generator_traits::value_to_yaml(msg.client_id, out);
    out << "\n";
  }

  // member: profile
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "profile: ";
    rosidl_generator_traits::value_to_yaml(msg.profile, out);
    out << "\n";
  }

  // member: resume_key
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "resume_key: ";
    rosidl_generator_traits::value_to_yaml(msg.resume_key, out);
    out << "\n";
  }

  // member: output_seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.output_seq, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Manage_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace arx_chassis_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use arx_chassis_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arx_chassis_interfaces::srv::Manage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  arx_chassis_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arx_chassis_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const arx_chassis_interfaces::srv::Manage_Request & msg)
{
  return arx_chassis_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<arx_chassis_interfaces::srv::Manage_Request>()
{
  return "arx_chassis_interfaces::srv::Manage_Request";
}

template<>
inline const char * name<arx_chassis_interfaces::srv::Manage_Request>()
{
  return "arx_chassis_interfaces/srv/Manage_Request";
}

template<>
struct has_fixed_size<arx_chassis_interfaces::srv::Manage_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arx_chassis_interfaces::srv::Manage_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arx_chassis_interfaces::srv::Manage_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'state'
#include "arx_chassis_interfaces/msg/detail/state__traits.hpp"

namespace arx_chassis_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Manage_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: client_id
  {
    out << "client_id: ";
    rosidl_generator_traits::value_to_yaml(msg.client_id, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    to_flow_style_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Manage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }

  // member: client_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "client_id: ";
    rosidl_generator_traits::value_to_yaml(msg.client_id, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state:\n";
    to_block_style_yaml(msg.state, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Manage_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace arx_chassis_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use arx_chassis_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arx_chassis_interfaces::srv::Manage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  arx_chassis_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arx_chassis_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const arx_chassis_interfaces::srv::Manage_Response & msg)
{
  return arx_chassis_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<arx_chassis_interfaces::srv::Manage_Response>()
{
  return "arx_chassis_interfaces::srv::Manage_Response";
}

template<>
inline const char * name<arx_chassis_interfaces::srv::Manage_Response>()
{
  return "arx_chassis_interfaces/srv/Manage_Response";
}

template<>
struct has_fixed_size<arx_chassis_interfaces::srv::Manage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arx_chassis_interfaces::srv::Manage_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arx_chassis_interfaces::srv::Manage_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace arx_chassis_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Manage_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Manage_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Manage_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace arx_chassis_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use arx_chassis_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arx_chassis_interfaces::srv::Manage_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  arx_chassis_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arx_chassis_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const arx_chassis_interfaces::srv::Manage_Event & msg)
{
  return arx_chassis_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<arx_chassis_interfaces::srv::Manage_Event>()
{
  return "arx_chassis_interfaces::srv::Manage_Event";
}

template<>
inline const char * name<arx_chassis_interfaces::srv::Manage_Event>()
{
  return "arx_chassis_interfaces/srv/Manage_Event";
}

template<>
struct has_fixed_size<arx_chassis_interfaces::srv::Manage_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arx_chassis_interfaces::srv::Manage_Event>
  : std::integral_constant<bool, has_bounded_size<arx_chassis_interfaces::srv::Manage_Request>::value && has_bounded_size<arx_chassis_interfaces::srv::Manage_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<arx_chassis_interfaces::srv::Manage_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<arx_chassis_interfaces::srv::Manage>()
{
  return "arx_chassis_interfaces::srv::Manage";
}

template<>
inline const char * name<arx_chassis_interfaces::srv::Manage>()
{
  return "arx_chassis_interfaces/srv/Manage";
}

template<>
struct has_fixed_size<arx_chassis_interfaces::srv::Manage>
  : std::integral_constant<
    bool,
    has_fixed_size<arx_chassis_interfaces::srv::Manage_Request>::value &&
    has_fixed_size<arx_chassis_interfaces::srv::Manage_Response>::value
  >
{
};

template<>
struct has_bounded_size<arx_chassis_interfaces::srv::Manage>
  : std::integral_constant<
    bool,
    has_bounded_size<arx_chassis_interfaces::srv::Manage_Request>::value &&
    has_bounded_size<arx_chassis_interfaces::srv::Manage_Response>::value
  >
{
};

template<>
struct is_service<arx_chassis_interfaces::srv::Manage>
  : std::true_type
{
};

template<>
struct is_service_request<arx_chassis_interfaces::srv::Manage_Request>
  : std::true_type
{
};

template<>
struct is_service_response<arx_chassis_interfaces::srv::Manage_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__TRAITS_HPP_
