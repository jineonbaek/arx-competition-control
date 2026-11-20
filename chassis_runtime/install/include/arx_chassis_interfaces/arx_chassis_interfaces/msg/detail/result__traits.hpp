// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arx_chassis_interfaces:msg/Result.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/result.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__TRAITS_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arx_chassis_interfaces/msg/detail/result__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'state'
#include "arx_chassis_interfaces/msg/detail/state__traits.hpp"

namespace arx_chassis_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: request_id
  {
    out << "request_id: ";
    rosidl_generator_traits::value_to_yaml(msg.request_id, out);
    out << ", ";
  }

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

  // member: output_seq
  {
    out << "output_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.output_seq, out);
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
  const Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request_id: ";
    rosidl_generator_traits::value_to_yaml(msg.request_id, out);
    out << "\n";
  }

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

  // member: output_seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.output_seq, out);
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

inline std::string to_yaml(const Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace arx_chassis_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use arx_chassis_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arx_chassis_interfaces::msg::Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  arx_chassis_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arx_chassis_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const arx_chassis_interfaces::msg::Result & msg)
{
  return arx_chassis_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arx_chassis_interfaces::msg::Result>()
{
  return "arx_chassis_interfaces::msg::Result";
}

template<>
inline const char * name<arx_chassis_interfaces::msg::Result>()
{
  return "arx_chassis_interfaces/msg/Result";
}

template<>
struct has_fixed_size<arx_chassis_interfaces::msg::Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arx_chassis_interfaces::msg::Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arx_chassis_interfaces::msg::Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__TRAITS_HPP_
