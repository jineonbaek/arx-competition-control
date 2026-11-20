// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arx_chassis_interfaces:srv/Manage.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/srv/manage.hpp"


#ifndef ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__STRUCT_HPP_
#define ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__arx_chassis_interfaces__srv__Manage_Request __attribute__((deprecated))
#else
# define DEPRECATED__arx_chassis_interfaces__srv__Manage_Request __declspec(deprecated)
#endif

namespace arx_chassis_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Manage_Request_
{
  using Type = Manage_Request_<ContainerAllocator>;

  explicit Manage_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operation = "";
      this->scheduler_instance = "";
      this->client_id = "";
      this->profile = "";
      this->resume_key = "";
      this->output_seq = 0ull;
    }
  }

  explicit Manage_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : operation(_alloc),
    scheduler_instance(_alloc),
    client_id(_alloc),
    profile(_alloc),
    resume_key(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operation = "";
      this->scheduler_instance = "";
      this->client_id = "";
      this->profile = "";
      this->resume_key = "";
      this->output_seq = 0ull;
    }
  }

  // field types and members
  using _operation_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operation_type operation;
  using _scheduler_instance_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _scheduler_instance_type scheduler_instance;
  using _client_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _client_id_type client_id;
  using _profile_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _profile_type profile;
  using _resume_key_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _resume_key_type resume_key;
  using _output_seq_type =
    uint64_t;
  _output_seq_type output_seq;

  // setters for named parameter idiom
  Type & set__operation(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operation = _arg;
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
  Type & set__profile(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->profile = _arg;
    return *this;
  }
  Type & set__resume_key(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->resume_key = _arg;
    return *this;
  }
  Type & set__output_seq(
    const uint64_t & _arg)
  {
    this->output_seq = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arx_chassis_interfaces__srv__Manage_Request
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arx_chassis_interfaces__srv__Manage_Request
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Manage_Request_ & other) const
  {
    if (this->operation != other.operation) {
      return false;
    }
    if (this->scheduler_instance != other.scheduler_instance) {
      return false;
    }
    if (this->client_id != other.client_id) {
      return false;
    }
    if (this->profile != other.profile) {
      return false;
    }
    if (this->resume_key != other.resume_key) {
      return false;
    }
    if (this->output_seq != other.output_seq) {
      return false;
    }
    return true;
  }
  bool operator!=(const Manage_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Manage_Request_

// alias to use template instance with default allocator
using Manage_Request =
  arx_chassis_interfaces::srv::Manage_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace arx_chassis_interfaces


// Include directives for member types
// Member 'state'
#include "arx_chassis_interfaces/msg/detail/state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arx_chassis_interfaces__srv__Manage_Response __attribute__((deprecated))
#else
# define DEPRECATED__arx_chassis_interfaces__srv__Manage_Response __declspec(deprecated)
#endif

namespace arx_chassis_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Manage_Response_
{
  using Type = Manage_Response_<ContainerAllocator>;

  explicit Manage_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->reason = "";
      this->client_id = "";
    }
  }

  explicit Manage_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reason(_alloc),
    client_id(_alloc),
    state(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
      this->reason = "";
      this->client_id = "";
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;
  using _client_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _client_id_type client_id;
  using _state_type =
    arx_chassis_interfaces::msg::State_<ContainerAllocator>;
  _state_type state;

  // setters for named parameter idiom
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
  Type & set__client_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->client_id = _arg;
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
    arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arx_chassis_interfaces__srv__Manage_Response
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arx_chassis_interfaces__srv__Manage_Response
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Manage_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    if (this->client_id != other.client_id) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const Manage_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Manage_Response_

// alias to use template instance with default allocator
using Manage_Response =
  arx_chassis_interfaces::srv::Manage_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace arx_chassis_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arx_chassis_interfaces__srv__Manage_Event __attribute__((deprecated))
#else
# define DEPRECATED__arx_chassis_interfaces__srv__Manage_Event __declspec(deprecated)
#endif

namespace arx_chassis_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Manage_Event_
{
  using Type = Manage_Event_<ContainerAllocator>;

  explicit Manage_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Manage_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arx_chassis_interfaces::srv::Manage_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arx_chassis_interfaces::srv::Manage_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arx_chassis_interfaces__srv__Manage_Event
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arx_chassis_interfaces__srv__Manage_Event
    std::shared_ptr<arx_chassis_interfaces::srv::Manage_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Manage_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Manage_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Manage_Event_

// alias to use template instance with default allocator
using Manage_Event =
  arx_chassis_interfaces::srv::Manage_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace arx_chassis_interfaces

namespace arx_chassis_interfaces
{

namespace srv
{

struct Manage
{
  using Request = arx_chassis_interfaces::srv::Manage_Request;
  using Response = arx_chassis_interfaces::srv::Manage_Response;
  using Event = arx_chassis_interfaces::srv::Manage_Event;
};

}  // namespace srv

}  // namespace arx_chassis_interfaces

#endif  // ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__STRUCT_HPP_
