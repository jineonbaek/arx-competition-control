// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arx_chassis_interfaces:msg/Result.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/result.h"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__STRUCT_H_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__STRUCT_H_

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
// Member 'reason'
#include "rosidl_runtime_c/string.h"
// Member 'state'
#include "arx_chassis_interfaces/msg/detail/state__struct.h"

/// Struct defined in msg/Result in the package arx_chassis_interfaces.
typedef struct arx_chassis_interfaces__msg__Result
{
  rosidl_runtime_c__String request_id;
  bool accepted;
  rosidl_runtime_c__String reason;
  uint64_t output_seq;
  arx_chassis_interfaces__msg__State state;
} arx_chassis_interfaces__msg__Result;

// Struct for a sequence of arx_chassis_interfaces__msg__Result.
typedef struct arx_chassis_interfaces__msg__Result__Sequence
{
  arx_chassis_interfaces__msg__Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arx_chassis_interfaces__msg__Result__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__RESULT__STRUCT_H_
