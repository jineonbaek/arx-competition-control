// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arx_chassis_interfaces:msg/State.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/state.h"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__STRUCT_H_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'scheduler_instance'
// Member 'driver_instance'
// Member 'client_id'
// Member 'source_id'
// Member 'task_id'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/State in the package arx_chassis_interfaces.
typedef struct arx_chassis_interfaces__msg__State
{
  rosidl_runtime_c__String scheduler_instance;
  bool ready;
  rosidl_runtime_c__String driver_instance;
  bool active;
  rosidl_runtime_c__String client_id;
  rosidl_runtime_c__String source_id;
  rosidl_runtime_c__String task_id;
  uint64_t generation;
  uint64_t output_seq;
  double expires_at;
  double sdk[3];
  bool maintenance;
  double applied_sdk[3];
  uint64_t applied_seq;
  rosidl_runtime_c__String reason;
} arx_chassis_interfaces__msg__State;

// Struct for a sequence of arx_chassis_interfaces__msg__State.
typedef struct arx_chassis_interfaces__msg__State__Sequence
{
  arx_chassis_interfaces__msg__State * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arx_chassis_interfaces__msg__State__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__STATE__STRUCT_H_
