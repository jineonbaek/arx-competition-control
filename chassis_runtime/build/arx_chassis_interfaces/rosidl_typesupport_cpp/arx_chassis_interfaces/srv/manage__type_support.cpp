// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from arx_chassis_interfaces:srv/Manage.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "arx_chassis_interfaces/srv/detail/manage__functions.h"
#include "arx_chassis_interfaces/srv/detail/manage__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace arx_chassis_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Manage_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Manage_Request_type_support_ids_t;

static const _Manage_Request_type_support_ids_t _Manage_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Manage_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Manage_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Manage_Request_type_support_symbol_names_t _Manage_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, arx_chassis_interfaces, srv, Manage_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arx_chassis_interfaces, srv, Manage_Request)),
  }
};

typedef struct _Manage_Request_type_support_data_t
{
  void * data[2];
} _Manage_Request_type_support_data_t;

static _Manage_Request_type_support_data_t _Manage_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Manage_Request_message_typesupport_map = {
  2,
  "arx_chassis_interfaces",
  &_Manage_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Manage_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Manage_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Manage_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Manage_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Request__get_type_hash,
  &arx_chassis_interfaces__srv__Manage_Request__get_type_description,
  &arx_chassis_interfaces__srv__Manage_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace arx_chassis_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<arx_chassis_interfaces::srv::Manage_Request>()
{
  return &::arx_chassis_interfaces::srv::rosidl_typesupport_cpp::Manage_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, arx_chassis_interfaces, srv, Manage_Request)() {
  return get_message_type_support_handle<arx_chassis_interfaces::srv::Manage_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__functions.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace arx_chassis_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Manage_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Manage_Response_type_support_ids_t;

static const _Manage_Response_type_support_ids_t _Manage_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Manage_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Manage_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Manage_Response_type_support_symbol_names_t _Manage_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, arx_chassis_interfaces, srv, Manage_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arx_chassis_interfaces, srv, Manage_Response)),
  }
};

typedef struct _Manage_Response_type_support_data_t
{
  void * data[2];
} _Manage_Response_type_support_data_t;

static _Manage_Response_type_support_data_t _Manage_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Manage_Response_message_typesupport_map = {
  2,
  "arx_chassis_interfaces",
  &_Manage_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Manage_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Manage_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Manage_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Manage_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Response__get_type_hash,
  &arx_chassis_interfaces__srv__Manage_Response__get_type_description,
  &arx_chassis_interfaces__srv__Manage_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace arx_chassis_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<arx_chassis_interfaces::srv::Manage_Response>()
{
  return &::arx_chassis_interfaces::srv::rosidl_typesupport_cpp::Manage_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, arx_chassis_interfaces, srv, Manage_Response)() {
  return get_message_type_support_handle<arx_chassis_interfaces::srv::Manage_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__functions.h"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace arx_chassis_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Manage_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Manage_Event_type_support_ids_t;

static const _Manage_Event_type_support_ids_t _Manage_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Manage_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Manage_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Manage_Event_type_support_symbol_names_t _Manage_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, arx_chassis_interfaces, srv, Manage_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arx_chassis_interfaces, srv, Manage_Event)),
  }
};

typedef struct _Manage_Event_type_support_data_t
{
  void * data[2];
} _Manage_Event_type_support_data_t;

static _Manage_Event_type_support_data_t _Manage_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Manage_Event_message_typesupport_map = {
  2,
  "arx_chassis_interfaces",
  &_Manage_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Manage_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Manage_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Manage_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Manage_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &arx_chassis_interfaces__srv__Manage_Event__get_type_hash,
  &arx_chassis_interfaces__srv__Manage_Event__get_type_description,
  &arx_chassis_interfaces__srv__Manage_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace arx_chassis_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<arx_chassis_interfaces::srv::Manage_Event>()
{
  return &::arx_chassis_interfaces::srv::rosidl_typesupport_cpp::Manage_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, arx_chassis_interfaces, srv, Manage_Event)() {
  return get_message_type_support_handle<arx_chassis_interfaces::srv::Manage_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace arx_chassis_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Manage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Manage_type_support_ids_t;

static const _Manage_type_support_ids_t _Manage_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Manage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Manage_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Manage_type_support_symbol_names_t _Manage_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, arx_chassis_interfaces, srv, Manage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arx_chassis_interfaces, srv, Manage)),
  }
};

typedef struct _Manage_type_support_data_t
{
  void * data[2];
} _Manage_type_support_data_t;

static _Manage_type_support_data_t _Manage_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Manage_service_typesupport_map = {
  2,
  "arx_chassis_interfaces",
  &_Manage_service_typesupport_ids.typesupport_identifier[0],
  &_Manage_service_typesupport_symbol_names.symbol_name[0],
  &_Manage_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Manage_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Manage_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<arx_chassis_interfaces::srv::Manage_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<arx_chassis_interfaces::srv::Manage_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<arx_chassis_interfaces::srv::Manage_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<arx_chassis_interfaces::srv::Manage>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<arx_chassis_interfaces::srv::Manage>,
  &arx_chassis_interfaces__srv__Manage__get_type_hash,
  &arx_chassis_interfaces__srv__Manage__get_type_description,
  &arx_chassis_interfaces__srv__Manage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace arx_chassis_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<arx_chassis_interfaces::srv::Manage>()
{
  return &::arx_chassis_interfaces::srv::rosidl_typesupport_cpp::Manage_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, arx_chassis_interfaces, srv, Manage)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<arx_chassis_interfaces::srv::Manage>();
}

#ifdef __cplusplus
}
#endif
