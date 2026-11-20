// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from arx_chassis_interfaces:srv/Manage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "arx_chassis_interfaces/srv/detail/manage__rosidl_typesupport_introspection_c.h"
#include "arx_chassis_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "arx_chassis_interfaces/srv/detail/manage__functions.h"
#include "arx_chassis_interfaces/srv/detail/manage__struct.h"


// Include directives for member types
// Member `operation`
// Member `scheduler_instance`
// Member `client_id`
// Member `profile`
// Member `resume_key`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arx_chassis_interfaces__srv__Manage_Request__init(message_memory);
}

void arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_fini_function(void * message_memory)
{
  arx_chassis_interfaces__srv__Manage_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_message_member_array[6] = {
  {
    "operation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__srv__Manage_Request, operation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scheduler_instance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__srv__Manage_Request, scheduler_instance),  // bytes offset in struct
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
    offsetof(arx_chassis_interfaces__srv__Manage_Request, client_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "profile",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__srv__Manage_Request, profile),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "resume_key",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__srv__Manage_Request, resume_key),  // bytes offset in struct
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
    offsetof(arx_chassis_interfaces__srv__Manage_Request, output_seq),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_message_members = {
  "arx_chassis_interfaces__srv",  // message namespace
  "Manage_Request",  // message name
  6,  // number of fields
  sizeof(arx_chassis_interfaces__srv__Manage_Request),
  false,  // has_any_key_member_
  arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_message_member_array,  // message members
  arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_message_type_support_handle = {
  0,
  &arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_message_members,
  get_message_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Request__get_type_hash,
  &arx_chassis_interfaces__srv__Manage_Request__get_type_description,
  &arx_chassis_interfaces__srv__Manage_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arx_chassis_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Request)() {
  if (!arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_message_type_support_handle.typesupport_identifier) {
    arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arx_chassis_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__functions.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__struct.h"


// Include directives for member types
// Member `reason`
// Member `client_id`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `state`
#include "arx_chassis_interfaces/msg/state.h"
// Member `state`
#include "arx_chassis_interfaces/msg/detail/state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arx_chassis_interfaces__srv__Manage_Response__init(message_memory);
}

void arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_fini_function(void * message_memory)
{
  arx_chassis_interfaces__srv__Manage_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_member_array[4] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__srv__Manage_Response, accepted),  // bytes offset in struct
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
    offsetof(arx_chassis_interfaces__srv__Manage_Response, reason),  // bytes offset in struct
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
    offsetof(arx_chassis_interfaces__srv__Manage_Response, client_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__srv__Manage_Response, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_members = {
  "arx_chassis_interfaces__srv",  // message namespace
  "Manage_Response",  // message name
  4,  // number of fields
  sizeof(arx_chassis_interfaces__srv__Manage_Response),
  false,  // has_any_key_member_
  arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_member_array,  // message members
  arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_type_support_handle = {
  0,
  &arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_members,
  get_message_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Response__get_type_hash,
  &arx_chassis_interfaces__srv__Manage_Response__get_type_description,
  &arx_chassis_interfaces__srv__Manage_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arx_chassis_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Response)() {
  arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, msg, State)();
  if (!arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_type_support_handle.typesupport_identifier) {
    arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "arx_chassis_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__functions.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "arx_chassis_interfaces/srv/manage.h"
// Member `request`
// Member `response`
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arx_chassis_interfaces__srv__Manage_Event__init(message_memory);
}

void arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_fini_function(void * message_memory)
{
  arx_chassis_interfaces__srv__Manage_Event__fini(message_memory);
}

size_t arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__size_function__Manage_Event__request(
  const void * untyped_member)
{
  const arx_chassis_interfaces__srv__Manage_Request__Sequence * member =
    (const arx_chassis_interfaces__srv__Manage_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_const_function__Manage_Event__request(
  const void * untyped_member, size_t index)
{
  const arx_chassis_interfaces__srv__Manage_Request__Sequence * member =
    (const arx_chassis_interfaces__srv__Manage_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_function__Manage_Event__request(
  void * untyped_member, size_t index)
{
  arx_chassis_interfaces__srv__Manage_Request__Sequence * member =
    (arx_chassis_interfaces__srv__Manage_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__fetch_function__Manage_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const arx_chassis_interfaces__srv__Manage_Request * item =
    ((const arx_chassis_interfaces__srv__Manage_Request *)
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_const_function__Manage_Event__request(untyped_member, index));
  arx_chassis_interfaces__srv__Manage_Request * value =
    (arx_chassis_interfaces__srv__Manage_Request *)(untyped_value);
  *value = *item;
}

void arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__assign_function__Manage_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  arx_chassis_interfaces__srv__Manage_Request * item =
    ((arx_chassis_interfaces__srv__Manage_Request *)
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_function__Manage_Event__request(untyped_member, index));
  const arx_chassis_interfaces__srv__Manage_Request * value =
    (const arx_chassis_interfaces__srv__Manage_Request *)(untyped_value);
  *item = *value;
}

bool arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__resize_function__Manage_Event__request(
  void * untyped_member, size_t size)
{
  arx_chassis_interfaces__srv__Manage_Request__Sequence * member =
    (arx_chassis_interfaces__srv__Manage_Request__Sequence *)(untyped_member);
  arx_chassis_interfaces__srv__Manage_Request__Sequence__fini(member);
  return arx_chassis_interfaces__srv__Manage_Request__Sequence__init(member, size);
}

size_t arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__size_function__Manage_Event__response(
  const void * untyped_member)
{
  const arx_chassis_interfaces__srv__Manage_Response__Sequence * member =
    (const arx_chassis_interfaces__srv__Manage_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_const_function__Manage_Event__response(
  const void * untyped_member, size_t index)
{
  const arx_chassis_interfaces__srv__Manage_Response__Sequence * member =
    (const arx_chassis_interfaces__srv__Manage_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_function__Manage_Event__response(
  void * untyped_member, size_t index)
{
  arx_chassis_interfaces__srv__Manage_Response__Sequence * member =
    (arx_chassis_interfaces__srv__Manage_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__fetch_function__Manage_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const arx_chassis_interfaces__srv__Manage_Response * item =
    ((const arx_chassis_interfaces__srv__Manage_Response *)
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_const_function__Manage_Event__response(untyped_member, index));
  arx_chassis_interfaces__srv__Manage_Response * value =
    (arx_chassis_interfaces__srv__Manage_Response *)(untyped_value);
  *value = *item;
}

void arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__assign_function__Manage_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  arx_chassis_interfaces__srv__Manage_Response * item =
    ((arx_chassis_interfaces__srv__Manage_Response *)
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_function__Manage_Event__response(untyped_member, index));
  const arx_chassis_interfaces__srv__Manage_Response * value =
    (const arx_chassis_interfaces__srv__Manage_Response *)(untyped_value);
  *item = *value;
}

bool arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__resize_function__Manage_Event__response(
  void * untyped_member, size_t size)
{
  arx_chassis_interfaces__srv__Manage_Response__Sequence * member =
    (arx_chassis_interfaces__srv__Manage_Response__Sequence *)(untyped_member);
  arx_chassis_interfaces__srv__Manage_Response__Sequence__fini(member);
  return arx_chassis_interfaces__srv__Manage_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arx_chassis_interfaces__srv__Manage_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(arx_chassis_interfaces__srv__Manage_Event, request),  // bytes offset in struct
    NULL,  // default value
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__size_function__Manage_Event__request,  // size() function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_const_function__Manage_Event__request,  // get_const(index) function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_function__Manage_Event__request,  // get(index) function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__fetch_function__Manage_Event__request,  // fetch(index, &value) function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__assign_function__Manage_Event__request,  // assign(index, value) function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__resize_function__Manage_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(arx_chassis_interfaces__srv__Manage_Event, response),  // bytes offset in struct
    NULL,  // default value
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__size_function__Manage_Event__response,  // size() function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_const_function__Manage_Event__response,  // get_const(index) function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__get_function__Manage_Event__response,  // get(index) function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__fetch_function__Manage_Event__response,  // fetch(index, &value) function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__assign_function__Manage_Event__response,  // assign(index, value) function pointer
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__resize_function__Manage_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_members = {
  "arx_chassis_interfaces__srv",  // message namespace
  "Manage_Event",  // message name
  3,  // number of fields
  sizeof(arx_chassis_interfaces__srv__Manage_Event),
  false,  // has_any_key_member_
  arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_member_array,  // message members
  arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_type_support_handle = {
  0,
  &arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_members,
  get_message_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Event__get_type_hash,
  &arx_chassis_interfaces__srv__Manage_Event__get_type_description,
  &arx_chassis_interfaces__srv__Manage_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arx_chassis_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Event)() {
  arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Request)();
  arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Response)();
  if (!arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_type_support_handle.typesupport_identifier) {
    arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "arx_chassis_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_service_members = {
  "arx_chassis_interfaces__srv",  // service namespace
  "Manage",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_Request_message_type_support_handle,
  NULL,  // response message
  // arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_Response_message_type_support_handle
  NULL  // event_message
  // arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_Response_message_type_support_handle
};


static rosidl_service_type_support_t arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_service_type_support_handle = {
  0,
  &arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_service_members,
  get_service_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Request__rosidl_typesupport_introspection_c__Manage_Request_message_type_support_handle,
  &arx_chassis_interfaces__srv__Manage_Response__rosidl_typesupport_introspection_c__Manage_Response_message_type_support_handle,
  &arx_chassis_interfaces__srv__Manage_Event__rosidl_typesupport_introspection_c__Manage_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    arx_chassis_interfaces,
    srv,
    Manage
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    arx_chassis_interfaces,
    srv,
    Manage
  ),
  &arx_chassis_interfaces__srv__Manage__get_type_hash,
  &arx_chassis_interfaces__srv__Manage__get_type_description,
  &arx_chassis_interfaces__srv__Manage__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arx_chassis_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage)(void) {
  if (!arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_service_type_support_handle.typesupport_identifier) {
    arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arx_chassis_interfaces, srv, Manage_Event)()->data;
  }

  return &arx_chassis_interfaces__srv__detail__manage__rosidl_typesupport_introspection_c__Manage_service_type_support_handle;
}
