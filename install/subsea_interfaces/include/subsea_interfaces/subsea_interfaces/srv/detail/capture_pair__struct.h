// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from subsea_interfaces:srv/CapturePair.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "subsea_interfaces/srv/capture_pair.h"


#ifndef SUBSEA_INTERFACES__SRV__DETAIL__CAPTURE_PAIR__STRUCT_H_
#define SUBSEA_INTERFACES__SRV__DETAIL__CAPTURE_PAIR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'session_id'
// Member 'output_dir'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CapturePair in the package subsea_interfaces.
typedef struct subsea_interfaces__srv__CapturePair_Request
{
  rosidl_runtime_c__String session_id;
  rosidl_runtime_c__String output_dir;
  int32_t jpeg_quality;
} subsea_interfaces__srv__CapturePair_Request;

// Struct for a sequence of subsea_interfaces__srv__CapturePair_Request.
typedef struct subsea_interfaces__srv__CapturePair_Request__Sequence
{
  subsea_interfaces__srv__CapturePair_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} subsea_interfaces__srv__CapturePair_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'cam0_path'
// Member 'cam1_path'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in srv/CapturePair in the package subsea_interfaces.
typedef struct subsea_interfaces__srv__CapturePair_Response
{
  bool success;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String cam0_path;
  rosidl_runtime_c__String cam1_path;
  builtin_interfaces__msg__Time stamp;
} subsea_interfaces__srv__CapturePair_Response;

// Struct for a sequence of subsea_interfaces__srv__CapturePair_Response.
typedef struct subsea_interfaces__srv__CapturePair_Response__Sequence
{
  subsea_interfaces__srv__CapturePair_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} subsea_interfaces__srv__CapturePair_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  subsea_interfaces__srv__CapturePair_Event__request__MAX_SIZE = 1
};
// response
enum
{
  subsea_interfaces__srv__CapturePair_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/CapturePair in the package subsea_interfaces.
typedef struct subsea_interfaces__srv__CapturePair_Event
{
  service_msgs__msg__ServiceEventInfo info;
  subsea_interfaces__srv__CapturePair_Request__Sequence request;
  subsea_interfaces__srv__CapturePair_Response__Sequence response;
} subsea_interfaces__srv__CapturePair_Event;

// Struct for a sequence of subsea_interfaces__srv__CapturePair_Event.
typedef struct subsea_interfaces__srv__CapturePair_Event__Sequence
{
  subsea_interfaces__srv__CapturePair_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} subsea_interfaces__srv__CapturePair_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SUBSEA_INTERFACES__SRV__DETAIL__CAPTURE_PAIR__STRUCT_H_
