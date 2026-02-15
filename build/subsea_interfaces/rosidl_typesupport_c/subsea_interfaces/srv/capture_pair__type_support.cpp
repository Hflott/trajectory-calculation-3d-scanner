// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from subsea_interfaces:srv/CapturePair.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "subsea_interfaces/srv/detail/capture_pair__struct.h"
#include "subsea_interfaces/srv/detail/capture_pair__type_support.h"
#include "subsea_interfaces/srv/detail/capture_pair__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace subsea_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _CapturePair_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CapturePair_Request_type_support_ids_t;

static const _CapturePair_Request_type_support_ids_t _CapturePair_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CapturePair_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CapturePair_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CapturePair_Request_type_support_symbol_names_t _CapturePair_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, subsea_interfaces, srv, CapturePair_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, subsea_interfaces, srv, CapturePair_Request)),
  }
};

typedef struct _CapturePair_Request_type_support_data_t
{
  void * data[2];
} _CapturePair_Request_type_support_data_t;

static _CapturePair_Request_type_support_data_t _CapturePair_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CapturePair_Request_message_typesupport_map = {
  2,
  "subsea_interfaces",
  &_CapturePair_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CapturePair_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CapturePair_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CapturePair_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CapturePair_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &subsea_interfaces__srv__CapturePair_Request__get_type_hash,
  &subsea_interfaces__srv__CapturePair_Request__get_type_description,
  &subsea_interfaces__srv__CapturePair_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace subsea_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, subsea_interfaces, srv, CapturePair_Request)() {
  return &::subsea_interfaces::srv::rosidl_typesupport_c::CapturePair_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__struct.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__type_support.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace subsea_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _CapturePair_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CapturePair_Response_type_support_ids_t;

static const _CapturePair_Response_type_support_ids_t _CapturePair_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CapturePair_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CapturePair_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CapturePair_Response_type_support_symbol_names_t _CapturePair_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, subsea_interfaces, srv, CapturePair_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, subsea_interfaces, srv, CapturePair_Response)),
  }
};

typedef struct _CapturePair_Response_type_support_data_t
{
  void * data[2];
} _CapturePair_Response_type_support_data_t;

static _CapturePair_Response_type_support_data_t _CapturePair_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CapturePair_Response_message_typesupport_map = {
  2,
  "subsea_interfaces",
  &_CapturePair_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CapturePair_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CapturePair_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CapturePair_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CapturePair_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &subsea_interfaces__srv__CapturePair_Response__get_type_hash,
  &subsea_interfaces__srv__CapturePair_Response__get_type_description,
  &subsea_interfaces__srv__CapturePair_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace subsea_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, subsea_interfaces, srv, CapturePair_Response)() {
  return &::subsea_interfaces::srv::rosidl_typesupport_c::CapturePair_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__struct.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__type_support.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace subsea_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _CapturePair_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CapturePair_Event_type_support_ids_t;

static const _CapturePair_Event_type_support_ids_t _CapturePair_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CapturePair_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CapturePair_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CapturePair_Event_type_support_symbol_names_t _CapturePair_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, subsea_interfaces, srv, CapturePair_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, subsea_interfaces, srv, CapturePair_Event)),
  }
};

typedef struct _CapturePair_Event_type_support_data_t
{
  void * data[2];
} _CapturePair_Event_type_support_data_t;

static _CapturePair_Event_type_support_data_t _CapturePair_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CapturePair_Event_message_typesupport_map = {
  2,
  "subsea_interfaces",
  &_CapturePair_Event_message_typesupport_ids.typesupport_identifier[0],
  &_CapturePair_Event_message_typesupport_symbol_names.symbol_name[0],
  &_CapturePair_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CapturePair_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CapturePair_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &subsea_interfaces__srv__CapturePair_Event__get_type_hash,
  &subsea_interfaces__srv__CapturePair_Event__get_type_description,
  &subsea_interfaces__srv__CapturePair_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace subsea_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, subsea_interfaces, srv, CapturePair_Event)() {
  return &::subsea_interfaces::srv::rosidl_typesupport_c::CapturePair_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace subsea_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _CapturePair_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CapturePair_type_support_ids_t;

static const _CapturePair_type_support_ids_t _CapturePair_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _CapturePair_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CapturePair_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CapturePair_type_support_symbol_names_t _CapturePair_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, subsea_interfaces, srv, CapturePair)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, subsea_interfaces, srv, CapturePair)),
  }
};

typedef struct _CapturePair_type_support_data_t
{
  void * data[2];
} _CapturePair_type_support_data_t;

static _CapturePair_type_support_data_t _CapturePair_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CapturePair_service_typesupport_map = {
  2,
  "subsea_interfaces",
  &_CapturePair_service_typesupport_ids.typesupport_identifier[0],
  &_CapturePair_service_typesupport_symbol_names.symbol_name[0],
  &_CapturePair_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CapturePair_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CapturePair_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &CapturePair_Request_message_type_support_handle,
  &CapturePair_Response_message_type_support_handle,
  &CapturePair_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    subsea_interfaces,
    srv,
    CapturePair
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    subsea_interfaces,
    srv,
    CapturePair
  ),
  &subsea_interfaces__srv__CapturePair__get_type_hash,
  &subsea_interfaces__srv__CapturePair__get_type_description,
  &subsea_interfaces__srv__CapturePair__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace subsea_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, subsea_interfaces, srv, CapturePair)() {
  return &::subsea_interfaces::srv::rosidl_typesupport_c::CapturePair_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
