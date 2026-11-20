// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arx_chassis_interfaces:msg/Command.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/command.h"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__STRUCT_H_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'request_id'
// Member 'scheduler_instance'
// Member 'client_id'
// Member 'operation'
// Member 'kind'
// Member 'source_id'
// Member 'task_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Command in the package arx_chassis_interfaces.
typedef struct arx_chassis_interfaces__msg__Command
{
  rosidl_runtime_c__String request_id;
  rosidl_runtime_c__String scheduler_instance;
  rosidl_runtime_c__String client_id;
  rosidl_runtime_c__String operation;
  rosidl_runtime_c__String kind;
  rosidl_runtime_c__String source_id;
  rosidl_runtime_c__String task_id;
  uint64_t generation;
  uint64_t seq;
  double issued_at;
  double ttl_s;
  double sdk[3];
} arx_chassis_interfaces__msg__Command;

// Struct for a sequence of arx_chassis_interfaces__msg__Command.
typedef struct arx_chassis_interfaces__msg__Command__Sequence
{
  arx_chassis_interfaces__msg__Command * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arx_chassis_interfaces__msg__Command__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__COMMAND__STRUCT_H_
