// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arx_chassis_interfaces:msg/Output.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "arx_chassis_interfaces/msg/output.h"


#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__STRUCT_H_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__STRUCT_H_

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
// Member 'reason'
// Member 'source_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Output in the package arx_chassis_interfaces.
typedef struct arx_chassis_interfaces__msg__Output
{
  rosidl_runtime_c__String scheduler_instance;
  rosidl_runtime_c__String driver_instance;
  uint64_t output_seq;
  double issued_at;
  double ttl_s;
  double sdk[3];
  rosidl_runtime_c__String reason;
  rosidl_runtime_c__String source_id;
} arx_chassis_interfaces__msg__Output;

// Struct for a sequence of arx_chassis_interfaces__msg__Output.
typedef struct arx_chassis_interfaces__msg__Output__Sequence
{
  arx_chassis_interfaces__msg__Output * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arx_chassis_interfaces__msg__Output__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__STRUCT_H_
