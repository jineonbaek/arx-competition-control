// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from arx_chassis_interfaces:srv/Manage.idl
// generated code does not contain a copyright notice
#include "arx_chassis_interfaces/srv/detail/manage__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "arx_chassis_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "arx_chassis_interfaces/srv/detail/manage__struct.h"
#include "arx_chassis_interfaces/srv/detail/manage__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // client_id, operation, profile, resume_key, scheduler_instance
#include "rosidl_runtime_c/string_functions.h"  // client_id, operation, profile, resume_key, scheduler_instance

// forward declare type support functions


using _Manage_Request__ros_msg_type = arx_chassis_interfaces__srv__Manage_Request;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_arx_chassis_interfaces__srv__Manage_Request(
  const arx_chassis_interfaces__srv__Manage_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: operation
  {
    const rosidl_runtime_c__String * str = &ros_message->operation;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: scheduler_instance
  {
    const rosidl_runtime_c__String * str = &ros_message->scheduler_instance;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: client_id
  {
    const rosidl_runtime_c__String * str = &ros_message->client_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: profile
  {
    const rosidl_runtime_c__String * str = &ros_message->profile;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: resume_key
  {
    const rosidl_runtime_c__String * str = &ros_message->resume_key;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: output_seq
  {
    cdr << ros_message->output_seq;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_deserialize_arx_chassis_interfaces__srv__Manage_Request(
  eprosima::fastcdr::Cdr & cdr,
  arx_chassis_interfaces__srv__Manage_Request * ros_message)
{
  // Field name: operation
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->operation.data) {
      rosidl_runtime_c__String__init(&ros_message->operation);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->operation,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'operation'\n");
      return false;
    }
  }

  // Field name: scheduler_instance
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->scheduler_instance.data) {
      rosidl_runtime_c__String__init(&ros_message->scheduler_instance);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->scheduler_instance,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'scheduler_instance'\n");
      return false;
    }
  }

  // Field name: client_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->client_id.data) {
      rosidl_runtime_c__String__init(&ros_message->client_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->client_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'client_id'\n");
      return false;
    }
  }

  // Field name: profile
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->profile.data) {
      rosidl_runtime_c__String__init(&ros_message->profile);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->profile,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'profile'\n");
      return false;
    }
  }

  // Field name: resume_key
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->resume_key.data) {
      rosidl_runtime_c__String__init(&ros_message->resume_key);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->resume_key,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'resume_key'\n");
      return false;
    }
  }

  // Field name: output_seq
  {
    cdr >> ros_message->output_seq;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_arx_chassis_interfaces__srv__Manage_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Manage_Request__ros_msg_type * ros_message = static_cast<const _Manage_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: operation
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->operation.size + 1);

  // Field name: scheduler_instance
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->scheduler_instance.size + 1);

  // Field name: client_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->client_id.size + 1);

  // Field name: profile
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->profile.size + 1);

  // Field name: resume_key
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->resume_key.size + 1);

  // Field name: output_seq
  {
    size_t item_size = sizeof(ros_message->output_seq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_arx_chassis_interfaces__srv__Manage_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: operation
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: scheduler_instance
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: client_id
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: profile
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: resume_key
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: output_seq
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = arx_chassis_interfaces__srv__Manage_Request;
    is_plain =
      (
      offsetof(DataType, output_seq) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_key_arx_chassis_interfaces__srv__Manage_Request(
  const arx_chassis_interfaces__srv__Manage_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: operation
  {
    const rosidl_runtime_c__String * str = &ros_message->operation;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: scheduler_instance
  {
    const rosidl_runtime_c__String * str = &ros_message->scheduler_instance;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: client_id
  {
    const rosidl_runtime_c__String * str = &ros_message->client_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: profile
  {
    const rosidl_runtime_c__String * str = &ros_message->profile;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: resume_key
  {
    const rosidl_runtime_c__String * str = &ros_message->resume_key;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: output_seq
  {
    cdr << ros_message->output_seq;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_key_arx_chassis_interfaces__srv__Manage_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Manage_Request__ros_msg_type * ros_message = static_cast<const _Manage_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: operation
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->operation.size + 1);

  // Field name: scheduler_instance
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->scheduler_instance.size + 1);

  // Field name: client_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->client_id.size + 1);

  // Field name: profile
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->profile.size + 1);

  // Field name: resume_key
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->resume_key.size + 1);

  // Field name: output_seq
  {
    size_t item_size = sizeof(ros_message->output_seq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_key_arx_chassis_interfaces__srv__Manage_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: operation
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: scheduler_instance
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: client_id
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: profile
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: resume_key
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: output_seq
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = arx_chassis_interfaces__srv__Manage_Request;
    is_plain =
      (
      offsetof(DataType, output_seq) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Manage_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const arx_chassis_interfaces__srv__Manage_Request * ros_message = static_cast<const arx_chassis_interfaces__srv__Manage_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_arx_chassis_interfaces__srv__Manage_Request(ros_message, cdr);
}

static bool _Manage_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  arx_chassis_interfaces__srv__Manage_Request * ros_message = static_cast<arx_chassis_interfaces__srv__Manage_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_arx_chassis_interfaces__srv__Manage_Request(cdr, ros_message);
}

static uint32_t _Manage_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_arx_chassis_interfaces__srv__Manage_Request(
      untyped_ros_message, 0));
}

static size_t _Manage_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_arx_chassis_interfaces__srv__Manage_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Manage_Request = {
  "arx_chassis_interfaces::srv",
  "Manage_Request",
  _Manage_Request__cdr_serialize,
  _Manage_Request__cdr_deserialize,
  _Manage_Request__get_serialized_size,
  _Manage_Request__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Manage_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Manage_Request,
  get_message_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Request__get_type_hash,
  &arx_chassis_interfaces__srv__Manage_Request__get_type_description,
  &arx_chassis_interfaces__srv__Manage_Request__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, srv, Manage_Request)() {
  return &_Manage_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "arx_chassis_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__struct.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "arx_chassis_interfaces/msg/detail/state__functions.h"  // state
// already included above
// #include "rosidl_runtime_c/string.h"  // client_id, reason
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // client_id, reason

// forward declare type support functions

bool cdr_serialize_arx_chassis_interfaces__msg__State(
  const arx_chassis_interfaces__msg__State * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_arx_chassis_interfaces__msg__State(
  eprosima::fastcdr::Cdr & cdr,
  arx_chassis_interfaces__msg__State * ros_message);

size_t get_serialized_size_arx_chassis_interfaces__msg__State(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_arx_chassis_interfaces__msg__State(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_arx_chassis_interfaces__msg__State(
  const arx_chassis_interfaces__msg__State * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_arx_chassis_interfaces__msg__State(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_arx_chassis_interfaces__msg__State(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, msg, State)();


using _Manage_Response__ros_msg_type = arx_chassis_interfaces__srv__Manage_Response;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_arx_chassis_interfaces__srv__Manage_Response(
  const arx_chassis_interfaces__srv__Manage_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: accepted
  {
    cdr << (ros_message->accepted ? true : false);
  }

  // Field name: reason
  {
    const rosidl_runtime_c__String * str = &ros_message->reason;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: client_id
  {
    const rosidl_runtime_c__String * str = &ros_message->client_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: state
  {
    cdr_serialize_arx_chassis_interfaces__msg__State(
      &ros_message->state, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_deserialize_arx_chassis_interfaces__srv__Manage_Response(
  eprosima::fastcdr::Cdr & cdr,
  arx_chassis_interfaces__srv__Manage_Response * ros_message)
{
  // Field name: accepted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->accepted = tmp ? true : false;
  }

  // Field name: reason
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->reason.data) {
      rosidl_runtime_c__String__init(&ros_message->reason);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->reason,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'reason'\n");
      return false;
    }
  }

  // Field name: client_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->client_id.data) {
      rosidl_runtime_c__String__init(&ros_message->client_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->client_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'client_id'\n");
      return false;
    }
  }

  // Field name: state
  {
    cdr_deserialize_arx_chassis_interfaces__msg__State(cdr, &ros_message->state);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_arx_chassis_interfaces__srv__Manage_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Manage_Response__ros_msg_type * ros_message = static_cast<const _Manage_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: accepted
  {
    size_t item_size = sizeof(ros_message->accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: reason
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->reason.size + 1);

  // Field name: client_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->client_id.size + 1);

  // Field name: state
  current_alignment += get_serialized_size_arx_chassis_interfaces__msg__State(
    &(ros_message->state), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_arx_chassis_interfaces__srv__Manage_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: accepted
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: reason
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: client_id
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: state
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_arx_chassis_interfaces__msg__State(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = arx_chassis_interfaces__srv__Manage_Response;
    is_plain =
      (
      offsetof(DataType, state) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_key_arx_chassis_interfaces__srv__Manage_Response(
  const arx_chassis_interfaces__srv__Manage_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: accepted
  {
    cdr << (ros_message->accepted ? true : false);
  }

  // Field name: reason
  {
    const rosidl_runtime_c__String * str = &ros_message->reason;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: client_id
  {
    const rosidl_runtime_c__String * str = &ros_message->client_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: state
  {
    cdr_serialize_key_arx_chassis_interfaces__msg__State(
      &ros_message->state, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_key_arx_chassis_interfaces__srv__Manage_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Manage_Response__ros_msg_type * ros_message = static_cast<const _Manage_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: accepted
  {
    size_t item_size = sizeof(ros_message->accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: reason
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->reason.size + 1);

  // Field name: client_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->client_id.size + 1);

  // Field name: state
  current_alignment += get_serialized_size_key_arx_chassis_interfaces__msg__State(
    &(ros_message->state), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_key_arx_chassis_interfaces__srv__Manage_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: accepted
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: reason
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: client_id
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: state
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_arx_chassis_interfaces__msg__State(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = arx_chassis_interfaces__srv__Manage_Response;
    is_plain =
      (
      offsetof(DataType, state) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Manage_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const arx_chassis_interfaces__srv__Manage_Response * ros_message = static_cast<const arx_chassis_interfaces__srv__Manage_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_arx_chassis_interfaces__srv__Manage_Response(ros_message, cdr);
}

static bool _Manage_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  arx_chassis_interfaces__srv__Manage_Response * ros_message = static_cast<arx_chassis_interfaces__srv__Manage_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_arx_chassis_interfaces__srv__Manage_Response(cdr, ros_message);
}

static uint32_t _Manage_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_arx_chassis_interfaces__srv__Manage_Response(
      untyped_ros_message, 0));
}

static size_t _Manage_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_arx_chassis_interfaces__srv__Manage_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Manage_Response = {
  "arx_chassis_interfaces::srv",
  "Manage_Response",
  _Manage_Response__cdr_serialize,
  _Manage_Response__cdr_deserialize,
  _Manage_Response__get_serialized_size,
  _Manage_Response__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Manage_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Manage_Response,
  get_message_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Response__get_type_hash,
  &arx_chassis_interfaces__srv__Manage_Response__get_type_description,
  &arx_chassis_interfaces__srv__Manage_Response__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, srv, Manage_Response)() {
  return &_Manage_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "arx_chassis_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__struct.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "service_msgs/msg/detail/service_event_info__functions.h"  // info

// forward declare type support functions

bool cdr_serialize_arx_chassis_interfaces__srv__Manage_Request(
  const arx_chassis_interfaces__srv__Manage_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_arx_chassis_interfaces__srv__Manage_Request(
  eprosima::fastcdr::Cdr & cdr,
  arx_chassis_interfaces__srv__Manage_Request * ros_message);

size_t get_serialized_size_arx_chassis_interfaces__srv__Manage_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_arx_chassis_interfaces__srv__Manage_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_arx_chassis_interfaces__srv__Manage_Request(
  const arx_chassis_interfaces__srv__Manage_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_arx_chassis_interfaces__srv__Manage_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_arx_chassis_interfaces__srv__Manage_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, srv, Manage_Request)();

bool cdr_serialize_arx_chassis_interfaces__srv__Manage_Response(
  const arx_chassis_interfaces__srv__Manage_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_arx_chassis_interfaces__srv__Manage_Response(
  eprosima::fastcdr::Cdr & cdr,
  arx_chassis_interfaces__srv__Manage_Response * ros_message);

size_t get_serialized_size_arx_chassis_interfaces__srv__Manage_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_arx_chassis_interfaces__srv__Manage_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_arx_chassis_interfaces__srv__Manage_Response(
  const arx_chassis_interfaces__srv__Manage_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_arx_chassis_interfaces__srv__Manage_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_arx_chassis_interfaces__srv__Manage_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, srv, Manage_Response)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_arx_chassis_interfaces
bool cdr_serialize_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_arx_chassis_interfaces
bool cdr_deserialize_service_msgs__msg__ServiceEventInfo(
  eprosima::fastcdr::Cdr & cdr,
  service_msgs__msg__ServiceEventInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_arx_chassis_interfaces
size_t get_serialized_size_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_arx_chassis_interfaces
size_t max_serialized_size_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_arx_chassis_interfaces
bool cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_arx_chassis_interfaces
size_t get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_arx_chassis_interfaces
size_t max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_arx_chassis_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, service_msgs, msg, ServiceEventInfo)();


using _Manage_Event__ros_msg_type = arx_chassis_interfaces__srv__Manage_Event;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_arx_chassis_interfaces__srv__Manage_Event(
  const arx_chassis_interfaces__srv__Manage_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_arx_chassis_interfaces__srv__Manage_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_arx_chassis_interfaces__srv__Manage_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_deserialize_arx_chassis_interfaces__srv__Manage_Event(
  eprosima::fastcdr::Cdr & cdr,
  arx_chassis_interfaces__srv__Manage_Event * ros_message)
{
  // Field name: info
  {
    cdr_deserialize_service_msgs__msg__ServiceEventInfo(cdr, &ros_message->info);
  }

  // Field name: request
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->request.data) {
      arx_chassis_interfaces__srv__Manage_Request__Sequence__fini(&ros_message->request);
    }
    if (!arx_chassis_interfaces__srv__Manage_Request__Sequence__init(&ros_message->request, size)) {
      fprintf(stderr, "failed to create array for field 'request'");
      return false;
    }
    auto array_ptr = ros_message->request.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_arx_chassis_interfaces__srv__Manage_Request(cdr, &array_ptr[i]);
    }
  }

  // Field name: response
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->response.data) {
      arx_chassis_interfaces__srv__Manage_Response__Sequence__fini(&ros_message->response);
    }
    if (!arx_chassis_interfaces__srv__Manage_Response__Sequence__init(&ros_message->response, size)) {
      fprintf(stderr, "failed to create array for field 'response'");
      return false;
    }
    auto array_ptr = ros_message->response.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_arx_chassis_interfaces__srv__Manage_Response(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_arx_chassis_interfaces__srv__Manage_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Manage_Event__ros_msg_type * ros_message = static_cast<const _Manage_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_arx_chassis_interfaces__srv__Manage_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_arx_chassis_interfaces__srv__Manage_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_arx_chassis_interfaces__srv__Manage_Event(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_arx_chassis_interfaces__srv__Manage_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_arx_chassis_interfaces__srv__Manage_Response(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = arx_chassis_interfaces__srv__Manage_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_key_arx_chassis_interfaces__srv__Manage_Event(
  const arx_chassis_interfaces__srv__Manage_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_arx_chassis_interfaces__srv__Manage_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_arx_chassis_interfaces__srv__Manage_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_key_arx_chassis_interfaces__srv__Manage_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Manage_Event__ros_msg_type * ros_message = static_cast<const _Manage_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_arx_chassis_interfaces__srv__Manage_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_arx_chassis_interfaces__srv__Manage_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_key_arx_chassis_interfaces__srv__Manage_Event(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_arx_chassis_interfaces__srv__Manage_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_arx_chassis_interfaces__srv__Manage_Response(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = arx_chassis_interfaces__srv__Manage_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Manage_Event__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const arx_chassis_interfaces__srv__Manage_Event * ros_message = static_cast<const arx_chassis_interfaces__srv__Manage_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_arx_chassis_interfaces__srv__Manage_Event(ros_message, cdr);
}

static bool _Manage_Event__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  arx_chassis_interfaces__srv__Manage_Event * ros_message = static_cast<arx_chassis_interfaces__srv__Manage_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_arx_chassis_interfaces__srv__Manage_Event(cdr, ros_message);
}

static uint32_t _Manage_Event__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_arx_chassis_interfaces__srv__Manage_Event(
      untyped_ros_message, 0));
}

static size_t _Manage_Event__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_arx_chassis_interfaces__srv__Manage_Event(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Manage_Event = {
  "arx_chassis_interfaces::srv",
  "Manage_Event",
  _Manage_Event__cdr_serialize,
  _Manage_Event__cdr_deserialize,
  _Manage_Event__get_serialized_size,
  _Manage_Event__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Manage_Event__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Manage_Event,
  get_message_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Event__get_type_hash,
  &arx_chassis_interfaces__srv__Manage_Event__get_type_description,
  &arx_chassis_interfaces__srv__Manage_Event__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, srv, Manage_Event)() {
  return &_Manage_Event__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "arx_chassis_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "arx_chassis_interfaces/srv/manage.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t Manage__callbacks = {
  "arx_chassis_interfaces::srv",
  "Manage",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, srv, Manage_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, srv, Manage_Response)(),
};

static rosidl_service_type_support_t Manage__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &Manage__callbacks,
  get_service_typesupport_handle_function,
  &_Manage_Request__type_support,
  &_Manage_Response__type_support,
  &_Manage_Event__type_support,
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

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, srv, Manage)() {
  return &Manage__handle;
}

#if defined(__cplusplus)
}
#endif
