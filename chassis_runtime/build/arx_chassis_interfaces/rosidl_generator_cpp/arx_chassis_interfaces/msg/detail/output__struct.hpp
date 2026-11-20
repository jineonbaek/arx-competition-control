// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arx_chassis_interfaces:msg/Output.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/output.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__STRUCT_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__arx_chassis_interfaces__msg__Output __attribute__((deprecated))
#else
# define DEPRECATED__arx_chassis_interfaces__msg__Output __declspec(deprecated)
#endif

namespace arx_chassis_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Output_
{
  using Type = Output_<ContainerAllocator>;

  explicit Output_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scheduler_instance = "";
      this->driver_instance = "";
      this->output_seq = 0ull;
      this->issued_at = 0.0;
      this->ttl_s = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->sdk.begin(), this->sdk.end(), 0.0);
      this->reason = "";
      this->source_id = "";
    }
  }

  explicit Output_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : scheduler_instance(_alloc),
    driver_instance(_alloc),
    sdk(_alloc),
    reason(_alloc),
    source_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scheduler_instance = "";
      this->driver_instance = "";
      this->output_seq = 0ull;
      this->issued_at = 0.0;
      this->ttl_s = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->sdk.begin(), this->sdk.end(), 0.0);
      this->reason = "";
      this->source_id = "";
    }
  }

  // field types and members
  using _scheduler_instance_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _scheduler_instance_type scheduler_instance;
  using _driver_instance_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _driver_instance_type driver_instance;
  using _output_seq_type =
    uint64_t;
  _output_seq_type output_seq;
  using _issued_at_type =
    double;
  _issued_at_type issued_at;
  using _ttl_s_type =
    double;
  _ttl_s_type ttl_s;
  using _sdk_type =
    std::array<double, 3>;
  _sdk_type sdk;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;
  using _source_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _source_id_type source_id;

  // setters for named parameter idiom
  Type & set__scheduler_instance(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->scheduler_instance = _arg;
    return *this;
  }
  Type & set__driver_instance(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->driver_instance = _arg;
    return *this;
  }
  Type & set__output_seq(
    const uint64_t & _arg)
  {
    this->output_seq = _arg;
    return *this;
  }
  Type & set__issued_at(
    const double & _arg)
  {
    this->issued_at = _arg;
    return *this;
  }
  Type & set__ttl_s(
    const double & _arg)
  {
    this->ttl_s = _arg;
    return *this;
  }
  Type & set__sdk(
    const std::array<double, 3> & _arg)
  {
    this->sdk = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }
  Type & set__source_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->source_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arx_chassis_interfaces::msg::Output_<ContainerAllocator> *;
  using ConstRawPtr =
    const arx_chassis_interfaces::msg::Output_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arx_chassis_interfaces::msg::Output_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arx_chassis_interfaces::msg::Output_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::msg::Output_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::msg::Output_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::msg::Output_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::msg::Output_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arx_chassis_interfaces::msg::Output_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arx_chassis_interfaces::msg::Output_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arx_chassis_interfaces__msg__Output
    std::shared_ptr<arx_chassis_interfaces::msg::Output_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arx_chassis_interfaces__msg__Output
    std::shared_ptr<arx_chassis_interfaces::msg::Output_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Output_ & other) const
  {
    if (this->scheduler_instance != other.scheduler_instance) {
      return false;
    }
    if (this->driver_instance != other.driver_instance) {
      return false;
    }
    if (this->output_seq != other.output_seq) {
      return false;
    }
    if (this->issued_at != other.issued_at) {
      return false;
    }
    if (this->ttl_s != other.ttl_s) {
      return false;
    }
    if (this->sdk != other.sdk) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    if (this->source_id != other.source_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Output_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Output_

// alias to use template instance with default allocator
using Output =
  arx_chassis_interfaces::msg::Output_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__STRUCT_HPP_
