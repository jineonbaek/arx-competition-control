// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from arx_chassis_interfaces:msg/Result.idl
// generated code does not contain a copyright notice
#include "arx_chassis_interfaces/msg/detail/result__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "arx_chassis_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "arx_chassis_interfaces/msg/detail/result__struct.h"
#include "arx_chassis_interfaces/msg/detail/result__functions.h"
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

#include "arx_chassis_interfaces/msg/detail/state__functions.h"  // state
#include "rosidl_runtime_c/string.h"  // reason, request_id
#include "rosidl_runtime_c/string_functions.h"  // reason, request_id

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


using _Result__ros_msg_type = arx_chassis_interfaces__msg__Result;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_arx_chassis_interfaces__msg__Result(
  const arx_chassis_interfaces__msg__Result * ros_message,
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

  // Field name: output_seq
  {
    cdr << ros_message->output_seq;
  }

  // Field name: state
  {
    cdr_serialize_arx_chassis_interfaces__msg__State(
      &ros_message->state, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_deserialize_arx_chassis_interfaces__msg__Result(
  eprosima::fastcdr::Cdr & cdr,
  arx_chassis_interfaces__msg__Result * ros_message)
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

  // Field name: output_seq
  {
    cdr >> ros_message->output_seq;
  }

  // Field name: state
  {
    cdr_deserialize_arx_chassis_interfaces__msg__State(cdr, &ros_message->state);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_arx_chassis_interfaces__msg__Result(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Result__ros_msg_type * ros_message = static_cast<const _Result__ros_msg_type *>(untyped_ros_message);
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

  // Field name: output_seq
  {
    size_t item_size = sizeof(ros_message->output_seq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: state
  current_alignment += get_serialized_size_arx_chassis_interfaces__msg__State(
    &(ros_message->state), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_arx_chassis_interfaces__msg__Result(
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

  // Field name: output_seq
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
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
    using DataType = arx_chassis_interfaces__msg__Result;
    is_plain =
      (
      offsetof(DataType, state) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_key_arx_chassis_interfaces__msg__Result(
  const arx_chassis_interfaces__msg__Result * ros_message,
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

  // Field name: output_seq
  {
    cdr << ros_message->output_seq;
  }

  // Field name: state
  {
    cdr_serialize_key_arx_chassis_interfaces__msg__State(
      &ros_message->state, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_key_arx_chassis_interfaces__msg__Result(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Result__ros_msg_type * ros_message = static_cast<const _Result__ros_msg_type *>(untyped_ros_message);
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

  // Field name: output_seq
  {
    size_t item_size = sizeof(ros_message->output_seq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: state
  current_alignment += get_serialized_size_key_arx_chassis_interfaces__msg__State(
    &(ros_message->state), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_key_arx_chassis_interfaces__msg__Result(
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

  // Field name: output_seq
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
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
    using DataType = arx_chassis_interfaces__msg__Result;
    is_plain =
      (
      offsetof(DataType, state) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Result__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const arx_chassis_interfaces__msg__Result * ros_message = static_cast<const arx_chassis_interfaces__msg__Result *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_arx_chassis_interfaces__msg__Result(ros_message, cdr);
}

static bool _Result__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  arx_chassis_interfaces__msg__Result * ros_message = static_cast<arx_chassis_interfaces__msg__Result *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_arx_chassis_interfaces__msg__Result(cdr, ros_message);
}

static uint32_t _Result__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_arx_chassis_interfaces__msg__Result(
      untyped_ros_message, 0));
}

static size_t _Result__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_arx_chassis_interfaces__msg__Result(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Result = {
  "arx_chassis_interfaces::msg",
  "Result",
  _Result__cdr_serialize,
  _Result__cdr_deserialize,
  _Result__get_serialized_size,
  _Result__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Result__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Result,
  get_message_typesupport_handle_function,
  &arx_chassis_interfaces__msg__Result__get_type_hash,
  &arx_chassis_interfaces__msg__Result__get_type_description,
  &arx_chassis_interfaces__msg__Result__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, msg, Result)() {
  return &_Result__type_support;
}

#if defined(__cplusplus)
}
#endif
