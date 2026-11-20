// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arx_chassis_interfaces:msg/State.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/state.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__TRAITS_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arx_chassis_interfaces/msg/detail/state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace arx_chassis_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const State & msg,
  std::ostream & out)
{
  out << "{";
  // member: scheduler_instance
  {
    out << "scheduler_instance: ";
    rosidl_generator_traits::value_to_yaml(msg.scheduler_instance, out);
    out << ", ";
  }

  // member: ready
  {
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
    out << ", ";
  }

  // member: driver_instance
  {
    out << "driver_instance: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_instance, out);
    out << ", ";
  }

  // member: active
  {
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << ", ";
  }

  // member: client_id
  {
    out << "client_id: ";
    rosidl_generator_traits::value_to_yaml(msg.client_id, out);
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

  // member: output_seq
  {
    out << "output_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.output_seq, out);
    out << ", ";
  }

  // member: expires_at
  {
    out << "expires_at: ";
    rosidl_generator_traits::value_to_yaml(msg.expires_at, out);
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

  // member: maintenance
  {
    out << "maintenance: ";
    rosidl_generator_traits::value_to_yaml(msg.maintenance, out);
    out << ", ";
  }

  // member: applied_sdk
  {
    if (msg.applied_sdk.size() == 0) {
      out << "applied_sdk: []";
    } else {
      out << "applied_sdk: [";
      size_t pending_items = msg.applied_sdk.size();
      for (auto item : msg.applied_sdk) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: applied_seq
  {
    out << "applied_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.applied_seq, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const State & msg,
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

  // member: ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
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

  // member: active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
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

  // member: output_seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.output_seq, out);
    out << "\n";
  }

  // member: expires_at
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expires_at: ";
    rosidl_generator_traits::value_to_yaml(msg.expires_at, out);
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

  // member: maintenance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "maintenance: ";
    rosidl_generator_traits::value_to_yaml(msg.maintenance, out);
    out << "\n";
  }

  // member: applied_sdk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.applied_sdk.size() == 0) {
      out << "applied_sdk: []\n";
    } else {
      out << "applied_sdk:\n";
      for (auto item : msg.applied_sdk) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: applied_seq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "applied_seq: ";
    rosidl_generator_traits::value_to_yaml(msg.applied_seq, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const State & msg, bool use_flow_style = false)
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
  const arx_chassis_interfaces::msg::State & msg,
  std::ostream & out, size_t indentation = 0)
{
  arx_chassis_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arx_chassis_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const arx_chassis_interfaces::msg::State & msg)
{
  return arx_chassis_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arx_chassis_interfaces::msg::State>()
{
  return "arx_chassis_interfaces::msg::State";
}

template<>
inline const char * name<arx_chassis_interfaces::msg::State>()
{
  return "arx_chassis_interfaces/msg/State";
}

template<>
struct has_fixed_size<arx_chassis_interfaces::msg::State>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<arx_chassis_interfaces::msg::State>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<arx_chassis_interfaces::msg::State>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__TRAITS_HPP_
