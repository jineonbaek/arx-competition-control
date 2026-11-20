// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arx_chassis_interfaces:srv/Manage.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/srv/manage.h"


#ifndef ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__STRUCT_H_
#define ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'operation'
// Member 'scheduler_instance'
// Member 'client_id'
// Member 'profile'
// Member 'resume_key'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Manage in the package arx_chassis_interfaces.
typedef struct arx_chassis_interfaces__srv__Manage_Request
{
  rosidl_runtime_c__String operation;
  rosidl_runtime_c__String scheduler_instance;
  rosidl_runtime_c__String client_id;
  rosidl_runtime_c__String profile;
  rosidl_runtime_c__String resume_key;
  uint64_t output_seq;
} arx_chassis_interfaces__srv__Manage_Request;

// Struct for a sequence of arx_chassis_interfaces__srv__Manage_Request.
typedef struct arx_chassis_interfaces__srv__Manage_Request__Sequence
{
  arx_chassis_interfaces__srv__Manage_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arx_chassis_interfaces__srv__Manage_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'reason'
// Member 'client_id'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'state'
#include "arx_chassis_interfaces/msg/detail/state__struct.h"

/// Struct defined in srv/Manage in the package arx_chassis_interfaces.
typedef struct arx_chassis_interfaces__srv__Manage_Response
{
  bool accepted;
  rosidl_runtime_c__String reason;
  rosidl_runtime_c__String client_id;
  arx_chassis_interfaces__msg__State state;
} arx_chassis_interfaces__srv__Manage_Response;

// Struct for a sequence of arx_chassis_interfaces__srv__Manage_Response.
typedef struct arx_chassis_interfaces__srv__Manage_Response__Sequence
{
  arx_chassis_interfaces__srv__Manage_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arx_chassis_interfaces__srv__Manage_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  arx_chassis_interfaces__srv__Manage_Event__request__MAX_SIZE = 1
};
// response
enum
{
  arx_chassis_interfaces__srv__Manage_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Manage in the package arx_chassis_interfaces.
typedef struct arx_chassis_interfaces__srv__Manage_Event
{
  service_msgs__msg__ServiceEventInfo info;
  arx_chassis_interfaces__srv__Manage_Request__Sequence request;
  arx_chassis_interfaces__srv__Manage_Response__Sequence response;
} arx_chassis_interfaces__srv__Manage_Event;

// Struct for a sequence of arx_chassis_interfaces__srv__Manage_Event.
typedef struct arx_chassis_interfaces__srv__Manage_Event__Sequence
{
  arx_chassis_interfaces__srv__Manage_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arx_chassis_interfaces__srv__Manage_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARX_CHASSIS_INTERFACES__SRV__DETAIL__MANAGE__STRUCT_H_
