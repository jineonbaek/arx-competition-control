// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arx_chassis_interfaces:msg/Command.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/command.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__TRAITS_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arx_chassis_interfaces/msg/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace arx_chassis_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Command & msg,
  std::ostream & out)
{
  out << "{";
  // member: request_id
  {
    out << "request_id: ";
    rosidl_generator_traits::value_to_yaml(msg.request_id, out);
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

  // member: operation
  {
    out << "operation: ";
    rosidl_generator_traits::value_to_yaml(msg.operation, out);
    out << ", ";
  }

  // member: kind
  {
    out << "kind: ";
    rosidl_generator_traits::value_to_yaml(msg.kind, out);
    out << ", ";
  }

  // member: source_id
  {
    out << "source_id: ";
    rosidl_generator_traits::value_to_yaml(msg.source_id, out);
    out << ", ";
  }

  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << ", ";
  }

  // member: generation
  {
    out << "generation: ";
    rosidl_generator_traits::value_to_yaml(msg.generation, out);
    out << ", ";
  }

  // member: seq
  {
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command & msg,
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

  // member: operation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation: ";
    rosidl_generator_traits::value_to_yaml(msg.operation, out);
    out << "\n";
  }

  // member: kind
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kind: ";
    rosidl_generator_traits::value_to_yaml(msg.kind, out);
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

  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }

  // member: generation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "generation: ";
    rosidl_generator_traits::value_to_yaml(msg.generation, out);
    out << "\n";
  }

  // member: seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seq: ";
    rosidl_generator_traits::value_to_yaml(msg.seq, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command & msg, bool use_flow_style = false)
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
  const arx_chassis_interfaces::msg::Command & msg,
  std::ostream & out, size_t indentation = 0)
{
  arx_chassis_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arx_chassis_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const arx_chassis_interfaces::msg::Command & msg)
{
  return arx_chassis_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arx_chassis_interfaces::msg::Command>()
{
  return "arx_chassis_interfaces::msg::Command";
}

template<>
inline const char * name<arx_chassis_interfaces::msg::Command>()
{
  return "arx_chassis_interfaces/msg/Command";
}

template<>
struct has_fixed_size<arx_chassis_interfaces::msg::Command>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arx_chassis_interfaces::msg::Command>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arx_chassis_interfaces::msg::Command>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__TRAITS_HPP_
