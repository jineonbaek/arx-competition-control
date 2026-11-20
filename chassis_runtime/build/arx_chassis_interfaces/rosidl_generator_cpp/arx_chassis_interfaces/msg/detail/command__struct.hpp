// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arx_chassis_interfaces:msg/Command.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/command.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__STRUCT_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__arx_chassis_interfaces__msg__Command __attribute__((deprecated))
#else
# define DEPRECATED__arx_chassis_interfaces__msg__Command __declspec(deprecated)
#endif

namespace arx_chassis_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Command_
{
  using Type = Command_<ContainerAllocator>;

  explicit Command_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_id = "";
      this->scheduler_instance = "";
      this->client_id = "";
      this->operation = "";
      this->kind = "";
      this->source_id = "";
      this->task_id = "";
      this->generation = 0ull;
      this->seq = 0ull;
      this->issued_at = 0.0;
      this->ttl_s = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->sdk.begin(), this->sdk.end(), 0.0);
    }
  }

  explicit Command_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request_id(_alloc),
    scheduler_instance(_alloc),
    client_id(_alloc),
    operation(_alloc),
    kind(_alloc),
    source_id(_alloc),
    task_id(_alloc),
    sdk(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request_id = "";
      this->scheduler_instance = "";
      this->client_id = "";
      this->operation = "";
      this->kind = "";
      this->source_id = "";
      this->task_id = "";
      this->generation = 0ull;
      this->seq = 0ull;
      this->issued_at = 0.0;
      this->ttl_s = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->sdk.begin(), this->sdk.end(), 0.0);
    }
  }

  // field types and members
  using _request_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _request_id_type request_id;
  using _scheduler_instance_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _scheduler_instance_type scheduler_instance;
  using _client_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _client_id_type client_id;
  using _operation_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operation_type operation;
  using _kind_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _kind_type kind;
  using _source_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _source_id_type source_id;
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _generation_type =
    uint64_t;
  _generation_type generation;
  using _seq_type =
    uint64_t;
  _seq_type seq;
  using _issued_at_type =
    double;
  _issued_at_type issued_at;
  using _ttl_s_type =
    double;
  _ttl_s_type ttl_s;
  using _sdk_type =
    std::array<double, 3>;
  _sdk_type sdk;

  // setters for named parameter idiom
  Type & set__request_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->request_id = _arg;
    return *this;
  }
  Type & set__scheduler_instance(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->scheduler_instance = _arg;
    return *this;
  }
  Type & set__client_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->client_id = _arg;
    return *this;
  }
  Type & set__operation(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operation = _arg;
    return *this;
  }
  Type & set__kind(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->kind = _arg;
    return *this;
  }
  Type & set__source_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->source_id = _arg;
    return *this;
  }
  Type & set__task_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__generation(
    const uint64_t & _arg)
  {
    this->generation = _arg;
    return *this;
  }
  Type & set__seq(
    const uint64_t & _arg)
  {
    this->seq = _arg;
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

  // constant declarations

  // pointer types
  using RawPtr =
    arx_chassis_interfaces::msg::Command_<ContainerAllocator> *;
  using ConstRawPtr =
    const arx_chassis_interfaces::msg::Command_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arx_chassis_interfaces::msg::Command_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arx_chassis_interfaces::msg::Command_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::msg::Command_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::msg::Command_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::msg::Command_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::msg::Command_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arx_chassis_interfaces::msg::Command_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arx_chassis_interfaces::msg::Command_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arx_chassis_interfaces__msg__Command
    std::shared_ptr<arx_chassis_interfaces::msg::Command_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arx_chassis_interfaces__msg__Command
    std::shared_ptr<arx_chassis_interfaces::msg::Command_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Command_ & other) const
  {
    if (this->request_id != other.request_id) {
      return false;
    }
    if (this->scheduler_instance != other.scheduler_instance) {
      return false;
    }
    if (this->client_id != other.client_id) {
      return false;
    }
    if (this->operation != other.operation) {
      return false;
    }
    if (this->kind != other.kind) {
      return false;
    }
    if (this->source_id != other.source_id) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->generation != other.generation) {
      return false;
    }
    if (this->seq != other.seq) {
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
    return true;
  }
  bool operator!=(const Command_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Command_

// alias to use template instance with default allocator
using Command =
  arx_chassis_interfaces::msg::Command_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__STRUCT_HPP_
