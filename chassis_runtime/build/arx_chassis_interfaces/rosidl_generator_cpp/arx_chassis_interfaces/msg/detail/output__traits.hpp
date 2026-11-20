// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arx_chassis_interfaces:msg/Output.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/output.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__TRAITS_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arx_chassis_interfaces/msg/detail/output__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace arx_chassis_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Output & msg,
  std::ostream & out)
{
  out << "{";
  // member: scheduler_instance
  {
    out << "scheduler_instance: ";
    rosidl_generator_traits::value_to_yaml(msg.scheduler_instance, out);
    out << ", ";
  }

  // member: driver_instance
  {
    out << "driver_instance: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_instance, out);
    out << ", ";
  }

  // member: output_seq
  {
    out << "output_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.output_seq, out);
    out << ", ";
  }

  // member: issued_at
  {
    out << "issued_at: ";
    rosidl_generator_traits::value_to_yaml(msg.issued_at, out);
    out << ", ";
  }

  // member: ttl_s
  {
    out << "ttl_s: ";
    rosidl_generator_traits::value_to_yaml(msg.ttl_s, out);
    out << ", ";
  }

  // member: sdk
  {
    if (msg.sdk.size() == 0) {
      out << "sdk: []";
    } else {
      out << "sdk: [";
      size_t pending_items = msg.sdk.size();
      for (auto item : msg.sdk) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: source_id
  {
    out << "source_id: ";
    rosidl_generator_traits::value_to_yaml(msg.source_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Output & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: scheduler_instance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scheduler_instance: ";
    rosidl_generator_traits::value_to_yaml(msg.scheduler_instance, out);
    out << "\n";
  }

  // member: driver_instance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "driver_instance: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_instance, out);
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

  // member: issued_at
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "issued_at: ";
    rosidl_generator_traits::value_to_yaml(msg.issued_at, out);
    out << "\n";
  }

  // member: ttl_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ttl_s: ";
    rosidl_generator_traits::value_to_yaml(msg.ttl_s, out);
    out << "\n";
  }

  // member: sdk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sdk.size() == 0) {
      out << "sdk: []\n";
    } else {
      out << "sdk:\n";
      for (auto item : msg.sdk) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

  // member: source_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source_id: ";
    rosidl_generator_traits::value_to_yaml(msg.source_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Output & msg, bool use_flow_style = false)
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
  const arx_chassis_interfaces::msg::Output & msg,
  std::ostream & out, size_t indentation = 0)
{
  arx_chassis_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arx_chassis_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const arx_chassis_interfaces::msg::Output & msg)
{
  return arx_chassis_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arx_chassis_interfaces::msg::Output>()
{
  return "arx_chassis_interfaces::msg::Output";
}

template<>
inline const char * name<arx_chassis_interfaces::msg::Output>()
{
  return "arx_chassis_interfaces/msg/Output";
}

template<>
struct has_fixed_size<arx_chassis_interfaces::msg::Output>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arx_chassis_interfaces::msg::Output>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arx_chassis_interfaces::msg::Output>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__TRAITS_HPP_
