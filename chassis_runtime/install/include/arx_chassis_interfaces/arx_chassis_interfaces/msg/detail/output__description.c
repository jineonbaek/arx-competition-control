// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from arx_chassis_interfaces:msg/Output.idl
// generated code does not contain a copyright notice

#include "arx_chassis_interfaces/msg/detail/output__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_arx_chassis_interfaces
const rosidl_type_hash_t *
arx_chassis_interfaces__msg__Output__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x75, 0x8e, 0xd4, 0x39, 0x13, 0x5b, 0xca, 0x49,
      0x20, 0x88, 0x87, 0xd0, 0x91, 0xc2, 0x22, 0x07,
      0x41, 0xb2, 0xc7, 0xea, 0x0c, 0x4b, 0x4b, 0xa2,
      0x5c, 0xeb, 0x40, 0x1d, 0xbc, 0x8d, 0x07, 0x73,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char arx_chassis_interfaces__msg__Output__TYPE_NAME[] = "arx_chassis_interfaces/msg/Output";

// Define type names, field names, and default values
static char arx_chassis_interfaces__msg__Output__FIELD_NAME__scheduler_instance[] = "scheduler_instance";
static char arx_chassis_interfaces__msg__Output__FIELD_NAME__driver_instance[] = "driver_instance";
static char arx_chassis_interfaces__msg__Output__FIELD_NAME__output_seq[] = "output_seq";
static char arx_chassis_interfaces__msg__Output__FIELD_NAME__issued_at[] = "issued_at";
static char arx_chassis_interfaces__msg__Output__FIELD_NAME__ttl_s[] = "ttl_s";
static char arx_chassis_interfaces__msg__Output__FIELD_NAME__sdk[] = "sdk";
static char arx_chassis_interfaces__msg__Output__FIELD_NAME__reason[] = "reason";
static char arx_chassis_interfaces__msg__Output__FIELD_NAME__source_id[] = "source_id";

static rosidl_runtime_c__type_description__Field arx_chassis_interfaces__msg__Output__FIELDS[] = {
  {
    {arx_chassis_interfaces__msg__Output__FIELD_NAME__scheduler_instance, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Output__FIELD_NAME__driver_instance, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Output__FIELD_NAME__output_seq, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Output__FIELD_NAME__issued_at, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Output__FIELD_NAME__ttl_s, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Output__FIELD_NAME__sdk, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Output__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Output__FIELD_NAME__source_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
arx_chassis_interfaces__msg__Output__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {arx_chassis_interfaces__msg__Output__TYPE_NAME, 33, 33},
      {arx_chassis_interfaces__msg__Output__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string scheduler_instance\n"
  "string driver_instance\n"
  "uint64 output_seq\n"
  "float64 issued_at\n"
  "float64 ttl_s\n"
  "float64[3] sdk\n"
  "string reason\n"
  "string source_id";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
arx_chassis_interfaces__msg__Output__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {arx_chassis_interfaces__msg__Output__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 145, 145},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
arx_chassis_interfaces__msg__Output__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *arx_chassis_interfaces__msg__Output__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
