// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from arx_chassis_interfaces:msg/Command.idl
// generated code does not contain a copyright notice

#include "arx_chassis_interfaces/msg/detail/command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_arx_chassis_interfaces
const rosidl_type_hash_t *
arx_chassis_interfaces__msg__Command__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0c, 0x7c, 0x08, 0x15, 0x24, 0x01, 0xc0, 0x79,
      0xee, 0x4f, 0x3f, 0x65, 0xfb, 0x45, 0x8a, 0xbe,
      0xc7, 0x09, 0x36, 0x01, 0xf5, 0x44, 0x73, 0x51,
      0x0c, 0xa2, 0x94, 0x23, 0x26, 0x8f, 0x9b, 0x2d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char arx_chassis_interfaces__msg__Command__TYPE_NAME[] = "arx_chassis_interfaces/msg/Command";

// Define type names, field names, and default values
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__request_id[] = "request_id";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__scheduler_instance[] = "scheduler_instance";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__client_id[] = "client_id";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__operation[] = "operation";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__kind[] = "kind";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__source_id[] = "source_id";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__task_id[] = "task_id";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__generation[] = "generation";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__seq[] = "seq";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__issued_at[] = "issued_at";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__ttl_s[] = "ttl_s";
static char arx_chassis_interfaces__msg__Command__FIELD_NAME__sdk[] = "sdk";

static rosidl_runtime_c__type_description__Field arx_chassis_interfaces__msg__Command__FIELDS[] = {
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__request_id, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__scheduler_instance, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__client_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__operation, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__kind, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__source_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__task_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__generation, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__seq, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__issued_at, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__ttl_s, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Command__FIELD_NAME__sdk, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
arx_chassis_interfaces__msg__Command__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {arx_chassis_interfaces__msg__Command__TYPE_NAME, 34, 34},
      {arx_chassis_interfaces__msg__Command__FIELDS, 12, 12},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string request_id\n"
  "string scheduler_instance\n"
  "string client_id\n"
  "string operation\n"
  "string kind\n"
  "string source_id\n"
  "string task_id\n"
  "uint64 generation\n"
  "uint64 seq\n"
  "float64 issued_at\n"
  "float64 ttl_s\n"
  "float64[3] sdk";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
arx_chassis_interfaces__msg__Command__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {arx_chassis_interfaces__msg__Command__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 198, 198},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
arx_chassis_interfaces__msg__Command__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *arx_chassis_interfaces__msg__Command__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
