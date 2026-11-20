// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from arx_chassis_interfaces:msg/Output.idl
// generated code does not contain a copyright notice
#ifndef ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "arx_chassis_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "arx_chassis_interfaces/msg/detail/output__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_arx_chassis_interfaces__msg__Output(
  const arx_chassis_interfaces__msg__Output * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_deserialize_arx_chassis_interfaces__msg__Output(
  eprosima::fastcdr::Cdr &,
  arx_chassis_interfaces__msg__Output * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_arx_chassis_interfaces__msg__Output(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_arx_chassis_interfaces__msg__Output(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
bool cdr_serialize_key_arx_chassis_interfaces__msg__Output(
  const arx_chassis_interfaces__msg__Output * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t get_serialized_size_key_arx_chassis_interfaces__msg__Output(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
size_t max_serialized_size_key_arx_chassis_interfaces__msg__Output(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_arx_chassis_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, arx_chassis_interfaces, msg, Output)();

#ifdef __cplusplus
}
#endif

#endif  // ARX_CHASSIS_INTERFACES__MSG__DETAIL__OUTPUT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
