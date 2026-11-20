// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from arx_chassis_interfaces:srv/Manage.idl
// generated code does not contain a copyright notice

#include "arx_chassis_interfaces/srv/detail/manage__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_arx_chassis_interfaces
const rosidl_type_hash_t *
arx_chassis_interfaces__srv__Manage__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3d, 0xf7, 0xaa, 0xe6, 0x73, 0x73, 0x7e, 0x45,
      0x0d, 0x18, 0xb2, 0xaa, 0xe4, 0x82, 0x9f, 0x08,
      0x2e, 0xeb, 0x66, 0xce, 0x61, 0xad, 0x27, 0x14,
      0x50, 0xbb, 0xfa, 0xc2, 0x22, 0xd0, 0xf3, 0x6f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_arx_chassis_interfaces
const rosidl_type_hash_t *
arx_chassis_interfaces__srv__Manage_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x70, 0x6c, 0x0a, 0xf9, 0x3e, 0xfb, 0x20, 0xf0,
      0xfb, 0xf5, 0x45, 0x82, 0x76, 0xc4, 0x53, 0x77,
      0x22, 0x5c, 0xf2, 0x52, 0x44, 0x2f, 0xa5, 0x39,
      0xfe, 0x9f, 0xf2, 0xa0, 0x02, 0xdd, 0x57, 0xad,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_arx_chassis_interfaces
const rosidl_type_hash_t *
arx_chassis_interfaces__srv__Manage_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf3, 0x28, 0x7a, 0xff, 0x7b, 0x3a, 0x94, 0xc9,
      0x30, 0x93, 0xbd, 0x28, 0xa8, 0x43, 0x4a, 0x99,
      0x91, 0xfd, 0x79, 0x02, 0xf7, 0x7e, 0x1b, 0xf7,
      0x2c, 0x21, 0x41, 0x22, 0xdc, 0x36, 0x21, 0x91,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_arx_chassis_interfaces
const rosidl_type_hash_t *
arx_chassis_interfaces__srv__Manage_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x45, 0x2b, 0x70, 0x26, 0x38, 0xe3, 0x3c, 0xb3,
      0xab, 0xb7, 0x68, 0xd6, 0x09, 0xa9, 0x9d, 0x57,
      0x3a, 0x12, 0x7c, 0x0b, 0x8b, 0x04, 0xe6, 0xb7,
      0x96, 0xc5, 0xab, 0xb3, 0x12, 0xa8, 0x96, 0x87,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "arx_chassis_interfaces/msg/detail/state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t arx_chassis_interfaces__msg__State__EXPECTED_HASH = {1, {
    0xeb, 0xcc, 0xec, 0xb4, 0xc2, 0xef, 0xfd, 0xac,
    0xff, 0xe7, 0x96, 0x99, 0x01, 0xf6, 0x84, 0xb6,
    0xcc, 0xce, 0x9e, 0xb7, 0xf6, 0xaf, 0x90, 0x61,
    0x90, 0xda, 0x6b, 0x96, 0x87, 0x17, 0x97, 0xf9,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char arx_chassis_interfaces__srv__Manage__TYPE_NAME[] = "arx_chassis_interfaces/srv/Manage";
static char arx_chassis_interfaces__msg__State__TYPE_NAME[] = "arx_chassis_interfaces/msg/State";
static char arx_chassis_interfaces__srv__Manage_Event__TYPE_NAME[] = "arx_chassis_interfaces/srv/Manage_Event";
static char arx_chassis_interfaces__srv__Manage_Request__TYPE_NAME[] = "arx_chassis_interfaces/srv/Manage_Request";
static char arx_chassis_interfaces__srv__Manage_Response__TYPE_NAME[] = "arx_chassis_interfaces/srv/Manage_Response";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char arx_chassis_interfaces__srv__Manage__FIELD_NAME__request_message[] = "request_message";
static char arx_chassis_interfaces__srv__Manage__FIELD_NAME__response_message[] = "response_message";
static char arx_chassis_interfaces__srv__Manage__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field arx_chassis_interfaces__srv__Manage__FIELDS[] = {
  {
    {arx_chassis_interfaces__srv__Manage__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {arx_chassis_interfaces__srv__Manage_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {arx_chassis_interfaces__srv__Manage_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {arx_chassis_interfaces__srv__Manage_Event__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription arx_chassis_interfaces__srv__Manage__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {arx_chassis_interfaces__msg__State__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Event__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
arx_chassis_interfaces__srv__Manage__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {arx_chassis_interfaces__srv__Manage__TYPE_NAME, 33, 33},
      {arx_chassis_interfaces__srv__Manage__FIELDS, 3, 3},
    },
    {arx_chassis_interfaces__srv__Manage__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&arx_chassis_interfaces__msg__State__EXPECTED_HASH, arx_chassis_interfaces__msg__State__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = arx_chassis_interfaces__msg__State__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = arx_chassis_interfaces__srv__Manage_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = arx_chassis_interfaces__srv__Manage_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = arx_chassis_interfaces__srv__Manage_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__operation[] = "operation";
static char arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__scheduler_instance[] = "scheduler_instance";
static char arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__client_id[] = "client_id";
static char arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__profile[] = "profile";
static char arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__resume_key[] = "resume_key";
static char arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__output_seq[] = "output_seq";

static rosidl_runtime_c__type_description__Field arx_chassis_interfaces__srv__Manage_Request__FIELDS[] = {
  {
    {arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__operation, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__scheduler_instance, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__client_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__profile, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__resume_key, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Request__FIELD_NAME__output_seq, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
arx_chassis_interfaces__srv__Manage_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {arx_chassis_interfaces__srv__Manage_Request__TYPE_NAME, 41, 41},
      {arx_chassis_interfaces__srv__Manage_Request__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char arx_chassis_interfaces__srv__Manage_Response__FIELD_NAME__accepted[] = "accepted";
static char arx_chassis_interfaces__srv__Manage_Response__FIELD_NAME__reason[] = "reason";
static char arx_chassis_interfaces__srv__Manage_Response__FIELD_NAME__client_id[] = "client_id";
static char arx_chassis_interfaces__srv__Manage_Response__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field arx_chassis_interfaces__srv__Manage_Response__FIELDS[] = {
  {
    {arx_chassis_interfaces__srv__Manage_Response__FIELD_NAME__accepted, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Response__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Response__FIELD_NAME__client_id, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Response__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {arx_chassis_interfaces__msg__State__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription arx_chassis_interfaces__srv__Manage_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {arx_chassis_interfaces__msg__State__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
arx_chassis_interfaces__srv__Manage_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {arx_chassis_interfaces__srv__Manage_Response__TYPE_NAME, 42, 42},
      {arx_chassis_interfaces__srv__Manage_Response__FIELDS, 4, 4},
    },
    {arx_chassis_interfaces__srv__Manage_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&arx_chassis_interfaces__msg__State__EXPECTED_HASH, arx_chassis_interfaces__msg__State__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = arx_chassis_interfaces__msg__State__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char arx_chassis_interfaces__srv__Manage_Event__FIELD_NAME__info[] = "info";
static char arx_chassis_interfaces__srv__Manage_Event__FIELD_NAME__request[] = "request";
static char arx_chassis_interfaces__srv__Manage_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field arx_chassis_interfaces__srv__Manage_Event__FIELDS[] = {
  {
    {arx_chassis_interfaces__srv__Manage_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {arx_chassis_interfaces__srv__Manage_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {arx_chassis_interfaces__srv__Manage_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription arx_chassis_interfaces__srv__Manage_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {arx_chassis_interfaces__msg__State__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {arx_chassis_interfaces__srv__Manage_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
arx_chassis_interfaces__srv__Manage_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {arx_chassis_interfaces__srv__Manage_Event__TYPE_NAME, 39, 39},
      {arx_chassis_interfaces__srv__Manage_Event__FIELDS, 3, 3},
    },
    {arx_chassis_interfaces__srv__Manage_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&arx_chassis_interfaces__msg__State__EXPECTED_HASH, arx_chassis_interfaces__msg__State__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = arx_chassis_interfaces__msg__State__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = arx_chassis_interfaces__srv__Manage_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = arx_chassis_interfaces__srv__Manage_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string operation\n"
  "string scheduler_instance\n"
  "string client_id\n"
  "string profile\n"
  "string resume_key\n"
  "uint64 output_seq\n"
  "---\n"
  "bool accepted\n"
  "string reason\n"
  "string client_id\n"
  "arx_chassis_interfaces/State state";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
arx_chassis_interfaces__srv__Manage__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {arx_chassis_interfaces__srv__Manage__TYPE_NAME, 33, 33},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 195, 195},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
arx_chassis_interfaces__srv__Manage_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {arx_chassis_interfaces__srv__Manage_Request__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
arx_chassis_interfaces__srv__Manage_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {arx_chassis_interfaces__srv__Manage_Response__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
arx_chassis_interfaces__srv__Manage_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {arx_chassis_interfaces__srv__Manage_Event__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
arx_chassis_interfaces__srv__Manage__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *arx_chassis_interfaces__srv__Manage__get_individual_type_description_source(NULL),
    sources[1] = *arx_chassis_interfaces__msg__State__get_individual_type_description_source(NULL);
    sources[2] = *arx_chassis_interfaces__srv__Manage_Event__get_individual_type_description_source(NULL);
    sources[3] = *arx_chassis_interfaces__srv__Manage_Request__get_individual_type_description_source(NULL);
    sources[4] = *arx_chassis_interfaces__srv__Manage_Response__get_individual_type_description_source(NULL);
    sources[5] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
arx_chassis_interfaces__srv__Manage_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *arx_chassis_interfaces__srv__Manage_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
arx_chassis_interfaces__srv__Manage_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *arx_chassis_interfaces__srv__Manage_Response__get_individual_type_description_source(NULL),
    sources[1] = *arx_chassis_interfaces__msg__State__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
arx_chassis_interfaces__srv__Manage_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *arx_chassis_interfaces__srv__Manage_Event__get_individual_type_description_source(NULL),
    sources[1] = *arx_chassis_interfaces__msg__State__get_individual_type_description_source(NULL);
    sources[2] = *arx_chassis_interfaces__srv__Manage_Request__get_individual_type_description_source(NULL);
    sources[3] = *arx_chassis_interfaces__srv__Manage_Response__get_individual_type_description_source(NULL);
    sources[4] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
