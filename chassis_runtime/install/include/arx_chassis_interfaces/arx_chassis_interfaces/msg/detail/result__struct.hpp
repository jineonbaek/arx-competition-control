// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arx_chassis_interfaces:msg/Result.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/result.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__STRUCT_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'state'
#include "arx_chassis_interfaces/msg/detail/state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arx_chassis_interfaces__msg__Result __attribute__((deprecated))
#else
# define DEPRECATED__arx_chassis_interfaces__msg__Result __declspec(deprecated)
#endif

namespace arx_chassis_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Result_
{
  using Type = Result_<ContainerAllocator>;

  explicit Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_id = "";
      this->accepted = false;
      this->reason = "";
      this->output_seq = 0ull;
    }
  }

  explicit Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request_id(_alloc),
    reason(_alloc),
    state(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_id = "";
      this->accepted = false;
      this->reason = "";
      this->output_seq = 0ull;
    }
  }

  // field types and members
  using _request_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _request_id_type request_id;
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;
  using _output_seq_type =
    uint64_t;
  _output_seq_type output_seq;
  using _state_type =
    arx_chassis_interfaces::msg::State_<ContainerAllocator>;
  _state_type state;

  // setters for named parameter idiom
  Type & set__request_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->request_id = _arg;
    return *this;
  }
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }
  Type & set__output_seq(
    const uint64_t & _arg)
  {
    this->output_seq = _arg;
    return *this;
  }
  Type & set__state(
    const arx_chassis_interfaces::msg::State_<ContainerAllocator> & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arx_chassis_interfaces::msg::Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const arx_chassis_interfaces::msg::Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arx_chassis_interfaces::msg::Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arx_chassis_interfaces::msg::Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::msg::Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::msg::Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::msg::Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::msg::Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arx_chassis_interfaces::msg::Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arx_chassis_interfaces::msg::Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arx_chassis_interfaces__msg__Result
    std::shared_ptr<arx_chassis_interfaces::msg::Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arx_chassis_interfaces__msg__Result
    std::shared_ptr<arx_chassis_interfaces::msg::Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Result_ & other) const
  {
    if (this->request_id != other.request_id) {
      return false;
    }
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    if (this->output_seq != other.output_seq) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Result_

// alias to use template instance with default allocator
using Result =
  arx_chassis_interfaces::msg::Result_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__STRUCT_HPP_
