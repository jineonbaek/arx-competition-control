// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from arx_chassis_interfaces:msg/Result.idl
// generated code does not contain a copyright notice

#include "arx_chassis_interfaces/msg/detail/result__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_arx_chassis_interfaces
const rosidl_type_hash_t *
arx_chassis_interfaces__msg__Result__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8e, 0x06, 0x45, 0xe5, 0xa9, 0x94, 0x84, 0xf5,
      0xa0, 0x8e, 0xc7, 0xd1, 0x9f, 0x4e, 0x05, 0x1e,
      0x88, 0x2d, 0xa8, 0xaa, 0x25, 0x80, 0xab, 0xda,
      0x2b, 0xd0, 0xbf, 0xa3, 0x5f, 0x35, 0xde, 0xbd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "arx_chassis_interfaces/msg/detail/state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t arx_chassis_interfaces__msg__State__EXPECTED_HASH = {1, {
    0xeb, 0xcc, 0xec, 0xb4, 0xc2, 0xef, 0xfd, 0xac,
    0xff, 0xe7, 0x96, 0x99, 0x01, 0xf6, 0x84, 0xb6,
    0xcc, 0xce, 0x9e, 0xb7, 0xf6, 0xaf, 0x90, 0x61,
    0x90, 0xda, 0x6b, 0x96, 0x87, 0x17, 0x97, 0xf9,
  }};
#endif

static char arx_chassis_interfaces__msg__Result__TYPE_NAME[] = "arx_chassis_interfaces/msg/Result";
static char arx_chassis_interfaces__msg__State__TYPE_NAME[] = "arx_chassis_interfaces/msg/State";

// Define type names, field names, and default values
static char arx_chassis_interfaces__msg__Result__FIELD_NAME__request_id[] = "request_id";
static char arx_chassis_interfaces__msg__Result__FIELD_NAME__accepted[] = "accepted";
static char arx_chassis_interfaces__msg__Result__FIELD_NAME__reason[] = "reason";
static char arx_chassis_interfaces__msg__Result__FIELD_NAME__output_seq[] = "output_seq";
static char arx_chassis_interfaces__msg__Result__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field arx_chassis_interfaces__msg__Result__FIELDS[] = {
  {
    {arx_chassis_interfaces__msg__Result__FIELD_NAME__request_id, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Result__FIELD_NAME__accepted, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Result__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Result__FIELD_NAME__output_seq, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__msg__Result__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {arx_chassis_interfaces__msg__State__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription arx_chassis_interfaces__msg__Result__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {arx_chassis_interfaces__msg__State__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
arx_chassis_interfaces__msg__Result__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {arx_chassis_interfaces__msg__Result__TYPE_NAME, 33, 33},
      {arx_chassis_interfaces__msg__Result__FIELDS, 5, 5},
    },
    {arx_chassis_interfaces__msg__Result__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&arx_chassis_interfaces__msg__State__EXPECTED_HASH, arx_chassis_interfaces__msg__State__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = arx_chassis_interfaces__msg__State__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string request_id\n"
  "bool accepted\n"
  "string reason\n"
  "uint64 output_seq\n"
  "arx_chassis_interfaces/State state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
arx_chassis_interfaces__msg__Result__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {arx_chassis_interfaces__msg__Result__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 99, 99},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
arx_chassis_interfaces__msg__Result__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *arx_chassis_interfaces__msg__Result__get_individual_type_description_source(NULL),
    sources[1] = *arx_chassis_interfaces__msg__State__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
