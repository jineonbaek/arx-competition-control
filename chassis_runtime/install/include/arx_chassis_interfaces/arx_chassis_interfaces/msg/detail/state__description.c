// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from arx_chassis_interfaces:msg/State.idl
// generated code does not contain a copyright notice

#include "arx_chassis_interfaces/msg/detail/state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_arx_chassis_interfaces
const rosidl_type_hash_t *
arx_chassis_interfaces__msg__State__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xeb, 0xcc, 0xec, 0xb4, 0xc2, 0xef, 0xfd, 0xac,
      0xff, 0xe7, 0x96, 0x99, 0x01, 0xf6, 0x84, 0xb6,
      0xcc, 0xce, 0x9e, 0xb7, 0xf6, 0xaf, 0x90, 0x61,
      0x90, 0xda, 0x6b, 0x96, 0x87, 0x17, 0x97, 0xf9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char arx_chassis_interfaces__msg__State__TYPE_NAME[] = "arx_chassis_interfaces/msg/State";

// Define type names, field names, and default values
static char arx_chassis_interfaces__msg__State__FIELD_NAME__scheduler_instance[] = "scheduler_instance";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__ready[] = "ready";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__driver_instance[] = "driver_instance";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__active[] = "active";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__client_id[] = "client_id";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__source_id[] = "source_id";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__task_id[] = "task_id";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__generation[] = "generation";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__output_seq[] = "output_seq";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__expires_at[] = "expires_at";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__sdk[] = "sdk";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__maintenance[] = "maintenance";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__applied_sdk[] = "applied_sdk";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__applied_seq[] = "applied_seq";
static char arx_chassis_interfaces__msg__State__FIELD_NAME__reason[] = "reason";

static rosidl_runtime_c__type_description__Field arx_chassis_interfaces__msg__State__FIELDS[] = {
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__scheduler_instance, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__ready, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__driver_instance, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__active, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__client_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__source_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__task_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__generation, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__output_seq, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__expires_at, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__sdk, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__maintenance, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__applied_sdk, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__applied_seq, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__State__FIELD_NAME__reason, 6, 6},
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
arx_chassis_interfaces__msg__State__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {arx_chassis_interfaces__msg__State__TYPE_NAME, 32, 32},
      {arx_chassis_interfaces__msg__State__FIELDS, 15, 15},
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
  "bool ready\n"
  "string driver_instance\n"
  "bool active\n"
  "string client_id\n"
  "string source_id\n"
  "string task_id\n"
  "uint64 generation\n"
  "uint64 output_seq\n"
  "float64 expires_at\n"
  "float64[3] sdk\n"
  "bool maintenance\n"
  "float64[3] applied_sdk\n"
  "uint64 applied_seq\n"
  "string reason";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
arx_chassis_interfaces__msg__State__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {arx_chassis_interfaces__msg__State__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 264, 264},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
arx_chassis_interfaces__msg__State__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *arx_chassis_interfaces__msg__State__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
