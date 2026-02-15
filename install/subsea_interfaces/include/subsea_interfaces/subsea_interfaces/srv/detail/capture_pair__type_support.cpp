// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from subsea_interfaces:srv/CapturePair.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "subsea_interfaces/srv/detail/capture_pair__functions.h"
#include "subsea_interfaces/srv/detail/capture_pair__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace subsea_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CapturePair_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) subsea_interfaces::srv::CapturePair_Request(_init);
}

void CapturePair_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<subsea_interfaces::srv::CapturePair_Request *>(message_memory);
  typed_message->~CapturePair_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CapturePair_Request_message_member_array[3] = {
  {
    "session_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Request, session_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "output_dir",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Request, output_dir),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "jpeg_quality",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Request, jpeg_quality),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CapturePair_Request_message_members = {
  "subsea_interfaces::srv",  // message namespace
  "CapturePair_Request",  // message name
  3,  // number of fields
  sizeof(subsea_interfaces::srv::CapturePair_Request),
  false,  // has_any_key_member_
  CapturePair_Request_message_member_array,  // message members
  CapturePair_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  CapturePair_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CapturePair_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CapturePair_Request_message_members,
  get_message_typesupport_handle_function,
  &subsea_interfaces__srv__CapturePair_Request__get_type_hash,
  &subsea_interfaces__srv__CapturePair_Request__get_type_description,
  &subsea_interfaces__srv__CapturePair_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace subsea_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<subsea_interfaces::srv::CapturePair_Request>()
{
  return &::subsea_interfaces::srv::rosidl_typesupport_introspection_cpp::CapturePair_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, subsea_interfaces, srv, CapturePair_Request)() {
  return &::subsea_interfaces::srv::rosidl_typesupport_introspection_cpp::CapturePair_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__functions.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace subsea_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CapturePair_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) subsea_interfaces::srv::CapturePair_Response(_init);
}

void CapturePair_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<subsea_interfaces::srv::CapturePair_Response *>(message_memory);
  typed_message->~CapturePair_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CapturePair_Response_message_member_array[5] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "message",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cam0_path",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Response, cam0_path),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cam1_path",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Response, cam1_path),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Response, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CapturePair_Response_message_members = {
  "subsea_interfaces::srv",  // message namespace
  "CapturePair_Response",  // message name
  5,  // number of fields
  sizeof(subsea_interfaces::srv::CapturePair_Response),
  false,  // has_any_key_member_
  CapturePair_Response_message_member_array,  // message members
  CapturePair_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  CapturePair_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CapturePair_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CapturePair_Response_message_members,
  get_message_typesupport_handle_function,
  &subsea_interfaces__srv__CapturePair_Response__get_type_hash,
  &subsea_interfaces__srv__CapturePair_Response__get_type_description,
  &subsea_interfaces__srv__CapturePair_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace subsea_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<subsea_interfaces::srv::CapturePair_Response>()
{
  return &::subsea_interfaces::srv::rosidl_typesupport_introspection_cpp::CapturePair_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, subsea_interfaces, srv, CapturePair_Response)() {
  return &::subsea_interfaces::srv::rosidl_typesupport_introspection_cpp::CapturePair_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__functions.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace subsea_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CapturePair_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) subsea_interfaces::srv::CapturePair_Event(_init);
}

void CapturePair_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<subsea_interfaces::srv::CapturePair_Event *>(message_memory);
  typed_message->~CapturePair_Event();
}

size_t size_function__CapturePair_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<subsea_interfaces::srv::CapturePair_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CapturePair_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<subsea_interfaces::srv::CapturePair_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__CapturePair_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<subsea_interfaces::srv::CapturePair_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__CapturePair_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const subsea_interfaces::srv::CapturePair_Request *>(
    get_const_function__CapturePair_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<subsea_interfaces::srv::CapturePair_Request *>(untyped_value);
  value = item;
}

void assign_function__CapturePair_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<subsea_interfaces::srv::CapturePair_Request *>(
    get_function__CapturePair_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const subsea_interfaces::srv::CapturePair_Request *>(untyped_value);
  item = value;
}

void resize_function__CapturePair_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<subsea_interfaces::srv::CapturePair_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CapturePair_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<subsea_interfaces::srv::CapturePair_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CapturePair_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<subsea_interfaces::srv::CapturePair_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__CapturePair_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<subsea_interfaces::srv::CapturePair_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__CapturePair_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const subsea_interfaces::srv::CapturePair_Response *>(
    get_const_function__CapturePair_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<subsea_interfaces::srv::CapturePair_Response *>(untyped_value);
  value = item;
}

void assign_function__CapturePair_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<subsea_interfaces::srv::CapturePair_Response *>(
    get_function__CapturePair_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const subsea_interfaces::srv::CapturePair_Response *>(untyped_value);
  item = value;
}

void resize_function__CapturePair_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<subsea_interfaces::srv::CapturePair_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CapturePair_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<subsea_interfaces::srv::CapturePair_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__CapturePair_Event__request,  // size() function pointer
    get_const_function__CapturePair_Event__request,  // get_const(index) function pointer
    get_function__CapturePair_Event__request,  // get(index) function pointer
    fetch_function__CapturePair_Event__request,  // fetch(index, &value) function pointer
    assign_function__CapturePair_Event__request,  // assign(index, value) function pointer
    resize_function__CapturePair_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<subsea_interfaces::srv::CapturePair_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(subsea_interfaces::srv::CapturePair_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__CapturePair_Event__response,  // size() function pointer
    get_const_function__CapturePair_Event__response,  // get_const(index) function pointer
    get_function__CapturePair_Event__response,  // get(index) function pointer
    fetch_function__CapturePair_Event__response,  // fetch(index, &value) function pointer
    assign_function__CapturePair_Event__response,  // assign(index, value) function pointer
    resize_function__CapturePair_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CapturePair_Event_message_members = {
  "subsea_interfaces::srv",  // message namespace
  "CapturePair_Event",  // message name
  3,  // number of fields
  sizeof(subsea_interfaces::srv::CapturePair_Event),
  false,  // has_any_key_member_
  CapturePair_Event_message_member_array,  // message members
  CapturePair_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  CapturePair_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CapturePair_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CapturePair_Event_message_members,
  get_message_typesupport_handle_function,
  &subsea_interfaces__srv__CapturePair_Event__get_type_hash,
  &subsea_interfaces__srv__CapturePair_Event__get_type_description,
  &subsea_interfaces__srv__CapturePair_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace subsea_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<subsea_interfaces::srv::CapturePair_Event>()
{
  return &::subsea_interfaces::srv::rosidl_typesupport_introspection_cpp::CapturePair_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, subsea_interfaces, srv, CapturePair_Event)() {
  return &::subsea_interfaces::srv::rosidl_typesupport_introspection_cpp::CapturePair_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__functions.h"
// already included above
// #include "subsea_interfaces/srv/detail/capture_pair__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace subsea_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers CapturePair_service_members = {
  "subsea_interfaces::srv",  // service namespace
  "CapturePair",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<subsea_interfaces::srv::CapturePair>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t CapturePair_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CapturePair_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<subsea_interfaces::srv::CapturePair_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<subsea_interfaces::srv::CapturePair_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<subsea_interfaces::srv::CapturePair_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<subsea_interfaces::srv::CapturePair>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<subsea_interfaces::srv::CapturePair>,
  &subsea_interfaces__srv__CapturePair__get_type_hash,
  &subsea_interfaces__srv__CapturePair__get_type_description,
  &subsea_interfaces__srv__CapturePair__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace subsea_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<subsea_interfaces::srv::CapturePair>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::subsea_interfaces::srv::rosidl_typesupport_introspection_cpp::CapturePair_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::subsea_interfaces::srv::CapturePair_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::subsea_interfaces::srv::CapturePair_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::subsea_interfaces::srv::CapturePair_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, subsea_interfaces, srv, CapturePair)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<subsea_interfaces::srv::CapturePair>();
}

#ifdef __cplusplus
}
#endif
