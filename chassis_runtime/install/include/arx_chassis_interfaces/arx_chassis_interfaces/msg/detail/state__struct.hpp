// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arx_chassis_interfaces:msg/State.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/state.hpp"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__STRUCT_HPP_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__arx_chassis_interfaces__msg__State __attribute__((deprecated))
#else
# define DEPRECATED__arx_chassis_interfaces__msg__State __declspec(deprecated)
#endif

namespace arx_chassis_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct State_
{
  using Type = State_<ContainerAllocator>;

  explicit State_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scheduler_instance = "";
      this->ready = false;
      this->driver_instance = "";
      this->active = false;
      this->client_id = "";
      this->source_id = "";
      this->task_id = "";
      this->generation = 0ull;
      this->output_seq = 0ull;
      this->expires_at = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->sdk.begin(), this->sdk.end(), 0.0);
      this->maintenance = false;
      std::fill<typename std::array<double, 3>::iterator, double>(this->applied_sdk.begin(), this->applied_sdk.end(), 0.0);
      this->applied_seq = 0ull;
      this->reason = "";
    }
  }

  explicit State_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : scheduler_instance(_alloc),
    driver_instance(_alloc),
    client_id(_alloc),
    source_id(_alloc),
    task_id(_alloc),
    sdk(_alloc),
    applied_sdk(_alloc),
    reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scheduler_instance = "";
      this->ready = false;
      this->driver_instance = "";
      this->active = false;
      this->client_id = "";
      this->source_id = "";
      this->task_id = "";
      this->generation = 0ull;
      this->output_seq = 0ull;
      this->expires_at = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->sdk.begin(), this->sdk.end(), 0.0);
      this->maintenance = false;
      std::fill<typename std::array<double, 3>::iterator, double>(this->applied_sdk.begin(), this->applied_sdk.end(), 0.0);
      this->applied_seq = 0ull;
      this->reason = "";
    }
  }

  // field types and members
  using _scheduler_instance_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _scheduler_instance_type scheduler_instance;
  using _ready_type =
    bool;
  _ready_type ready;
  using _driver_instance_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _driver_instance_type driver_instance;
  using _active_type =
    bool;
  _active_type active;
  using _client_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _client_id_type client_id;
  using _source_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _source_id_type source_id;
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _generation_type =
    uint64_t;
  _generation_type generation;
  using _output_seq_type =
    uint64_t;
  _output_seq_type output_seq;
  using _expires_at_type =
    double;
  _expires_at_type expires_at;
  using _sdk_type =
    std::array<double, 3>;
  _sdk_type sdk;
  using _maintenance_type =
    bool;
  _maintenance_type maintenance;
  using _applied_sdk_type =
    std::array<double, 3>;
  _applied_sdk_type applied_sdk;
  using _applied_seq_type =
    uint64_t;
  _applied_seq_type applied_seq;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__scheduler_instance(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->scheduler_instance = _arg;
    return *this;
  }
  Type & set__ready(
    const bool & _arg)
  {
    this->ready = _arg;
    return *this;
  }
  Type & set__driver_instance(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->driver_instance = _arg;
    return *this;
  }
  Type & set__active(
    const bool & _arg)
  {
    this->active = _arg;
    return *this;
  }
  Type & set__client_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->client_id = _arg;
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
  Type & set__output_seq(
    const uint64_t & _arg)
  {
    this->output_seq = _arg;
    return *this;
  }
  Type & set__expires_at(
    const double & _arg)
  {
    this->expires_at = _arg;
    return *this;
  }
  Type & set__sdk(
    const std::array<double, 3> & _arg)
  {
    this->sdk = _arg;
    return *this;
  }
  Type & set__maintenance(
    const bool & _arg)
  {
    this->maintenance = _arg;
    return *this;
  }
  Type & set__applied_sdk(
    const std::array<double, 3> & _arg)
  {
    this->applied_sdk = _arg;
    return *this;
  }
  Type & set__applied_seq(
    const uint64_t & _arg)
  {
    this->applied_seq = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arx_chassis_interfaces::msg::State_<ContainerAllocator> *;
  using ConstRawPtr =
    const arx_chassis_interfaces::msg::State_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arx_chassis_interfaces::msg::State_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arx_chassis_interfaces::msg::State_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::msg::State_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::msg::State_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::msg::State_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::msg::State_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arx_chassis_interfaces::msg::State_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arx_chassis_interfaces::msg::State_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arx_chassis_interfaces__msg__State
    std::shared_ptr<arx_chassis_interfaces::msg::State_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arx_chassis_interfaces__msg__State
    std::shared_ptr<arx_chassis_interfaces::msg::State_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const State_ & other) const
  {
    if (this->scheduler_instance != other.scheduler_instance) {
      return false;
    }
    if (this->ready != other.ready) {
      return false;
    }
    if (this->driver_instance != other.driver_instance) {
      return false;
    }
    if (this->active != other.active) {
      return false;
    }
    if (this->client_id != other.client_id) {
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
    if (this->output_seq != other.output_seq) {
      return false;
    }
    if (this->expires_at != other.expires_at) {
      return false;
    }
    if (this->sdk != other.sdk) {
      return false;
    }
    if (this->maintenance != other.maintenance) {
      return false;
    }
    if (this->applied_sdk != other.applied_sdk) {
      return false;
    }
    if (this->applied_seq != other.applied_seq) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const State_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct State_

// alias to use template instance with default allocator
using State =
  arx_chassis_interfaces::msg::State_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__STRUCT_HPP_
