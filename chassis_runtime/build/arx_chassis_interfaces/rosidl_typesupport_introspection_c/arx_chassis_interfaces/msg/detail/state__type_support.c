// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from arx_chassis_interfaces:msg/State.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "arx_chassis_interfaces/msg/detail/state__rosidl_typesupport_introspection_c.h"
#include "arx_chassis_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "arx_chassis_interfaces/msg/detail/state__functions.h"
#include "arx_chassis_interfaces/msg/detail/state__struct.h"


// Include directives for member types
// Member `scheduler_instance`
// Member `driver_instance`
// Member `client_id`
// Member `source_id`
// Member `task_id`
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arx_chassis_interfaces__msg__State__init(message_memory);
}

void arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_fini_function(void * message_memory)
{
  arx_chassis_interfaces__msg__State__fini(message_memory);
}

size_t arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__size_function__State__sdk(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_const_function__State__sdk(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_function__State__sdk(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__fetch_function__State__sdk(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_const_function__State__sdk(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__assign_function__State__sdk(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_function__State__sdk(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__size_function__State__applied_sdk(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_const_function__State__applied_sdk(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_function__State__applied_sdk(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__fetch_function__State__applied_sdk(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_const_function__State__applied_sdk(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__assign_function__State__applied_sdk(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_function__State__applied_sdk(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_message_member_array[15] = {
  {
    "scheduler_instance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, scheduler_instance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ready",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, ready),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "driver_instance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, driver_instance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "active",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, active),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "client_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, client_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "source_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, source_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, task_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "generation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, generation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "output_seq",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, output_seq),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "expires_at",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, expires_at),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sdk",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, sdk),  // bytes offset in struct
    NULL,  // default value
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__size_function__State__sdk,  // size() function pointer
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_const_function__State__sdk,  // get_const(index) function pointer
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_function__State__sdk,  // get(index) function pointer
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__fetch_function__State__sdk,  // fetch(index, &value) function pointer
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__assign_function__State__sdk,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "maintenance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, maintenance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "applied_sdk",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, applied_sdk),  // bytes offset in struct
    NULL,  // default value
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__size_function__State__applied_sdk,  // size() function pointer
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_const_function__State__applied_sdk,  // get_const(index) function pointer
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__get_function__State__applied_sdk,  // get(index) function pointer
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__fetch_function__State__applied_sdk,  // fetch(index, &value) function pointer
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__assign_function__State__applied_sdk,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "applied_seq",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, applied_seq),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__msg__State, reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_message_members = {
  "arx_chassis_interfaces__msg",  // message namespace
  "State",  // message name
  15,  // number of fields
  sizeof(arx_chassis_interfaces__msg__State),
  false,  // has_any_key_member_
  arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_message_member_array,  // message members
  arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_init_function,  // function to initialize message memory (memory has to be allocated)
  arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_message_type_support_handle = {
  0,
  &arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_message_members,
  get_message_typesupport_handle_function,
  &arx_chassis_interfaces__msg__State__get_type_hash,
  &arx_chassis_interfaces__msg__State__get_type_description,
  &arx_chassis_interfaces__msg__State__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arx_chassis_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, msg, State)() {
  if (!arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_message_type_support_handle.typesupport_identifier) {
    arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arx_chassis_interfaces__msg__State__rosidl_typesupport_introspection_c__State_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
