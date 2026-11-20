// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from arx_chassis_interfaces:msg/Command.idl
// generated code does not contain a copyright notice
#include "arx_chassis_interfaces/msg/detail/command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "arx_chassis_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "arx_chassis_interfaces/msg/detail/command__struct.h"
#include "arx_chassis_interfaces/msg/detail/command__functions.h"
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

#include "rosidl_runtime_c/string.h"  // client_id, kind, operation, request_id, scheduler_instance, source_id, task_id
#include "rosidl_runtime_c/string_functions.h"  // client_id, kind, operation, request_id, scheduler_instance, source_id, task_id

// forward declare type support functions


using _Command__ros_msg_type = arx_chassis_interfaces__msg__Command;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_arx_chassis_interfaces__msg__Command(
  const arx_chassis_interfaces__msg__Command * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: request_id
  {
    const rosidl_runtime_c__String * str = &ros_message->request_id;
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

  // Field name: kind
  {
    const rosidl_runtime_c__String * str = &ros_message->kind;
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

  // Field name: source_id
  {
    const rosidl_runtime_c__String * str = &ros_message->source_id;
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

  // Field name: task_id
  {
    const rosidl_runtime_c__String * str = &ros_message->task_id;
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

  // Field name: generation
  {
    cdr << ros_message->generation;
  }

  // Field name: seq
  {
    cdr << ros_message->seq;
  }

  // Field name: issued_at
  {
    cdr << ros_message->issued_at;
  }

  // Field name: ttl_s
  {
    cdr << ros_message->ttl_s;
  }

  // Field name: sdk
  {
    size_t size = 3;
    auto array_ptr = ros_message->sdk;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_deserialize_arx_chassis_interfaces__msg__Command(
  eprosima::fastcdr::Cdr & cdr,
  arx_chassis_interfaces__msg__Command * ros_message)
{
  // Field name: request_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->request_id.data) {
      rosidl_runtime_c__String__init(&ros_message->request_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->request_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'request_id'\n");
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

  // Field name: kind
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->kind.data) {
      rosidl_runtime_c__String__init(&ros_message->kind);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->kind,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'kind'\n");
      return false;
    }
  }

  // Field name: source_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->source_id.data) {
      rosidl_runtime_c__String__init(&ros_message->source_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->source_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'source_id'\n");
      return false;
    }
  }

  // Field name: task_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->task_id.data) {
      rosidl_runtime_c__String__init(&ros_message->task_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->task_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'task_id'\n");
      return false;
    }
  }

  // Field name: generation
  {
    cdr >> ros_message->generation;
  }

  // Field name: seq
  {
    cdr >> ros_message->seq;
  }

  // Field name: issued_at
  {
    cdr >> ros_message->issued_at;
  }

  // Field name: ttl_s
  {
    cdr >> ros_message->ttl_s;
  }

  // Field name: sdk
  {
    size_t size = 3;
    auto array_ptr = ros_message->sdk;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_arx_chassis_interfaces__msg__Command(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Command__ros_msg_type * ros_message = static_cast<const _Command__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: request_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->request_id.size + 1);

  // Field name: scheduler_instance
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->scheduler_instance.size + 1);

  // Field name: client_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->client_id.size + 1);

  // Field name: operation
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->operation.size + 1);

  // Field name: kind
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->kind.size + 1);

  // Field name: source_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->source_id.size + 1);

  // Field name: task_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->task_id.size + 1);

  // Field name: generation
  {
    size_t item_size = sizeof(ros_message->generation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: seq
  {
    size_t item_size = sizeof(ros_message->seq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: issued_at
  {
    size_t item_size = sizeof(ros_message->issued_at);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ttl_s
  {
    size_t item_size = sizeof(ros_message->ttl_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: sdk
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->sdk;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_arx_chassis_interfaces__msg__Command(
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

  // Field name: request_id
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

  // Field name: kind
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

  // Field name: source_id
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

  // Field name: task_id
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

  // Field name: generation
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: seq
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: issued_at
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: ttl_s
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: sdk
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = arx_chassis_interfaces__msg__Command;
    is_plain =
      (
      offsetof(DataType, sdk) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_key_arx_chassis_interfaces__msg__Command(
  const arx_chassis_interfaces__msg__Command * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: request_id
  {
    const rosidl_runtime_c__String * str = &ros_message->request_id;
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

  // Field name: kind
  {
    const rosidl_runtime_c__String * str = &ros_message->kind;
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

  // Field name: source_id
  {
    const rosidl_runtime_c__String * str = &ros_message->source_id;
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

  // Field name: task_id
  {
    const rosidl_runtime_c__String * str = &ros_message->task_id;
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

  // Field name: generation
  {
    cdr << ros_message->generation;
  }

  // Field name: seq
  {
    cdr << ros_message->seq;
  }

  // Field name: issued_at
  {
    cdr << ros_message->issued_at;
  }

  // Field name: ttl_s
  {
    cdr << ros_message->ttl_s;
  }

  // Field name: sdk
  {
    size_t size = 3;
    auto array_ptr = ros_message->sdk;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_key_arx_chassis_interfaces__msg__Command(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Command__ros_msg_type * ros_message = static_cast<const _Command__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: request_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->request_id.size + 1);

  // Field name: scheduler_instance
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->scheduler_instance.size + 1);

  // Field name: client_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->client_id.size + 1);

  // Field name: operation
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->operation.size + 1);

  // Field name: kind
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->kind.size + 1);

  // Field name: source_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->source_id.size + 1);

  // Field name: task_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->task_id.size + 1);

  // Field name: generation
  {
    size_t item_size = sizeof(ros_message->generation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: seq
  {
    size_t item_size = sizeof(ros_message->seq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: issued_at
  {
    size_t item_size = sizeof(ros_message->issued_at);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ttl_s
  {
    size_t item_size = sizeof(ros_message->ttl_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: sdk
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->sdk;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_key_arx_chassis_interfaces__msg__Command(
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
  // Field name: request_id
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

  // Field name: kind
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

  // Field name: source_id
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

  // Field name: task_id
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

  // Field name: generation
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: seq
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: issued_at
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: ttl_s
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: sdk
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = arx_chassis_interfaces__msg__Command;
    is_plain =
      (
      offsetof(DataType, sdk) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Command__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const arx_chassis_interfaces__msg__Command * ros_message = static_cast<const arx_chassis_interfaces__msg__Command *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_arx_chassis_interfaces__msg__Command(ros_message, cdr);
}

static bool _Command__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  arx_chassis_interfaces__msg__Command * ros_message = static_cast<arx_chassis_interfaces__msg__Command *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_arx_chassis_interfaces__msg__Command(cdr, ros_message);
}

static uint32_t _Command__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_arx_chassis_interfaces__msg__Command(
      untyped_ros_message, 0));
}

static size_t _Command__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_arx_chassis_interfaces__msg__Command(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Command = {
  "arx_chassis_interfaces::msg",
  "Command",
  _Command__cdr_serialize,
  _Command__cdr_deserialize,
  _Command__get_serialized_size,
  _Command__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Command__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Command,
  get_message_typesupport_handle_function,
  &arx_chassis_interfaces__msg__Command__get_type_hash,
  &arx_chassis_interfaces__msg__Command__get_type_description,
  &arx_chassis_interfaces__msg__Command__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, msg, Command)() {
  return &_Command__type_support;
}

#if defined(__cplusplus)
}
#endif
