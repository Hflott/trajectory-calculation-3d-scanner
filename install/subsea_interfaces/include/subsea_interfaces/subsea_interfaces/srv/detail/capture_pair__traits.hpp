// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from subsea_interfaces:srv/CapturePair.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "subsea_interfaces/srv/capture_pair.hpp"


#ifndef SUBSEA_INTERFACES__SRV__DETAIL__CAPTURE_PAIR__TRAITS_HPP_
#define SUBSEA_INTERFACES__SRV__DETAIL__CAPTURE_PAIR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "subsea_interfaces/srv/detail/capture_pair__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace subsea_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CapturePair_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: session_id
  {
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << ", ";
  }

  // member: output_dir
  {
    out << "output_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.output_dir, out);
    out << ", ";
  }

  // member: jpeg_quality
  {
    out << "jpeg_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.jpeg_quality, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CapturePair_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: session_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << "\n";
  }

  // member: output_dir
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.output_dir, out);
    out << "\n";
  }

  // member: jpeg_quality
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jpeg_quality: ";
    rosidl_generator_traits::value_to_yaml(msg.jpeg_quality, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CapturePair_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace subsea_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use subsea_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const subsea_interfaces::srv::CapturePair_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  subsea_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use subsea_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const subsea_interfaces::srv::CapturePair_Request & msg)
{
  return subsea_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<subsea_interfaces::srv::CapturePair_Request>()
{
  return "subsea_interfaces::srv::CapturePair_Request";
}

template<>
inline const char * name<subsea_interfaces::srv::CapturePair_Request>()
{
  return "subsea_interfaces/srv/CapturePair_Request";
}

template<>
struct has_fixed_size<subsea_interfaces::srv::CapturePair_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<subsea_interfaces::srv::CapturePair_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<subsea_interfaces::srv::CapturePair_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace subsea_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CapturePair_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: cam0_path
  {
    out << "cam0_path: ";
    rosidl_generator_traits::value_to_yaml(msg.cam0_path, out);
    out << ", ";
  }

  // member: cam1_path
  {
    out << "cam1_path: ";
    rosidl_generator_traits::value_to_yaml(msg.cam1_path, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CapturePair_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: cam0_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cam0_path: ";
    rosidl_generator_traits::value_to_yaml(msg.cam0_path, out);
    out << "\n";
  }

  // member: cam1_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cam1_path: ";
    rosidl_generator_traits::value_to_yaml(msg.cam1_path, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CapturePair_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace subsea_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use subsea_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const subsea_interfaces::srv::CapturePair_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  subsea_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use subsea_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const subsea_interfaces::srv::CapturePair_Response & msg)
{
  return subsea_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<subsea_interfaces::srv::CapturePair_Response>()
{
  return "subsea_interfaces::srv::CapturePair_Response";
}

template<>
inline const char * name<subsea_interfaces::srv::CapturePair_Response>()
{
  return "subsea_interfaces/srv/CapturePair_Response";
}

template<>
struct has_fixed_size<subsea_interfaces::srv::CapturePair_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<subsea_interfaces::srv::CapturePair_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<subsea_interfaces::srv::CapturePair_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace subsea_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CapturePair_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CapturePair_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CapturePair_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace subsea_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use subsea_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const subsea_interfaces::srv::CapturePair_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  subsea_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use subsea_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const subsea_interfaces::srv::CapturePair_Event & msg)
{
  return subsea_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<subsea_interfaces::srv::CapturePair_Event>()
{
  return "subsea_interfaces::srv::CapturePair_Event";
}

template<>
inline const char * name<subsea_interfaces::srv::CapturePair_Event>()
{
  return "subsea_interfaces/srv/CapturePair_Event";
}

template<>
struct has_fixed_size<subsea_interfaces::srv::CapturePair_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<subsea_interfaces::srv::CapturePair_Event>
  : std::integral_constant<bool, has_bounded_size<service_msgs::msg::ServiceEventInfo>::value && has_bounded_size<subsea_interfaces::srv::CapturePair_Request>::value && has_bounded_size<subsea_interfaces::srv::CapturePair_Response>::value> {};

template<>
struct is_message<subsea_interfaces::srv::CapturePair_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<subsea_interfaces::srv::CapturePair>()
{
  return "subsea_interfaces::srv::CapturePair";
}

template<>
inline const char * name<subsea_interfaces::srv::CapturePair>()
{
  return "subsea_interfaces/srv/CapturePair";
}

template<>
struct has_fixed_size<subsea_interfaces::srv::CapturePair>
  : std::integral_constant<
    bool,
    has_fixed_size<subsea_interfaces::srv::CapturePair_Request>::value &&
    has_fixed_size<subsea_interfaces::srv::CapturePair_Response>::value
  >
{
};

template<>
struct has_bounded_size<subsea_interfaces::srv::CapturePair>
  : std::integral_constant<
    bool,
    has_bounded_size<subsea_interfaces::srv::CapturePair_Request>::value &&
    has_bounded_size<subsea_interfaces::srv::CapturePair_Response>::value
  >
{
};

template<>
struct is_service<subsea_interfaces::srv::CapturePair>
  : std::true_type
{
};

template<>
struct is_service_request<subsea_interfaces::srv::CapturePair_Request>
  : std::true_type
{
};

template<>
struct is_service_response<subsea_interfaces::srv::CapturePair_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SUBSEA_INTERFACES__SRV__DETAIL__CAPTURE_PAIR__TRAITS_HPP_
