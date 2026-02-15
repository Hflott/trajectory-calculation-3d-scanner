// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from subsea_interfaces:srv/CapturePair.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "subsea_interfaces/srv/capture_pair.hpp"


#ifndef SUBSEA_INTERFACES__SRV__DETAIL__CAPTURE_PAIR__BUILDER_HPP_
#define SUBSEA_INTERFACES__SRV__DETAIL__CAPTURE_PAIR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "subsea_interfaces/srv/detail/capture_pair__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace subsea_interfaces
{

namespace srv
{

namespace builder
{

class Init_CapturePair_Request_jpeg_quality
{
public:
  explicit Init_CapturePair_Request_jpeg_quality(::subsea_interfaces::srv::CapturePair_Request & msg)
  : msg_(msg)
  {}
  ::subsea_interfaces::srv::CapturePair_Request jpeg_quality(::subsea_interfaces::srv::CapturePair_Request::_jpeg_quality_type arg)
  {
    msg_.jpeg_quality = std::move(arg);
    return std::move(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Request msg_;
};

class Init_CapturePair_Request_output_dir
{
public:
  explicit Init_CapturePair_Request_output_dir(::subsea_interfaces::srv::CapturePair_Request & msg)
  : msg_(msg)
  {}
  Init_CapturePair_Request_jpeg_quality output_dir(::subsea_interfaces::srv::CapturePair_Request::_output_dir_type arg)
  {
    msg_.output_dir = std::move(arg);
    return Init_CapturePair_Request_jpeg_quality(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Request msg_;
};

class Init_CapturePair_Request_session_id
{
public:
  Init_CapturePair_Request_session_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CapturePair_Request_output_dir session_id(::subsea_interfaces::srv::CapturePair_Request::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return Init_CapturePair_Request_output_dir(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::subsea_interfaces::srv::CapturePair_Request>()
{
  return subsea_interfaces::srv::builder::Init_CapturePair_Request_session_id();
}

}  // namespace subsea_interfaces


namespace subsea_interfaces
{

namespace srv
{

namespace builder
{

class Init_CapturePair_Response_stamp
{
public:
  explicit Init_CapturePair_Response_stamp(::subsea_interfaces::srv::CapturePair_Response & msg)
  : msg_(msg)
  {}
  ::subsea_interfaces::srv::CapturePair_Response stamp(::subsea_interfaces::srv::CapturePair_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Response msg_;
};

class Init_CapturePair_Response_cam1_path
{
public:
  explicit Init_CapturePair_Response_cam1_path(::subsea_interfaces::srv::CapturePair_Response & msg)
  : msg_(msg)
  {}
  Init_CapturePair_Response_stamp cam1_path(::subsea_interfaces::srv::CapturePair_Response::_cam1_path_type arg)
  {
    msg_.cam1_path = std::move(arg);
    return Init_CapturePair_Response_stamp(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Response msg_;
};

class Init_CapturePair_Response_cam0_path
{
public:
  explicit Init_CapturePair_Response_cam0_path(::subsea_interfaces::srv::CapturePair_Response & msg)
  : msg_(msg)
  {}
  Init_CapturePair_Response_cam1_path cam0_path(::subsea_interfaces::srv::CapturePair_Response::_cam0_path_type arg)
  {
    msg_.cam0_path = std::move(arg);
    return Init_CapturePair_Response_cam1_path(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Response msg_;
};

class Init_CapturePair_Response_message
{
public:
  explicit Init_CapturePair_Response_message(::subsea_interfaces::srv::CapturePair_Response & msg)
  : msg_(msg)
  {}
  Init_CapturePair_Response_cam0_path message(::subsea_interfaces::srv::CapturePair_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_CapturePair_Response_cam0_path(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Response msg_;
};

class Init_CapturePair_Response_success
{
public:
  Init_CapturePair_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CapturePair_Response_message success(::subsea_interfaces::srv::CapturePair_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CapturePair_Response_message(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::subsea_interfaces::srv::CapturePair_Response>()
{
  return subsea_interfaces::srv::builder::Init_CapturePair_Response_success();
}

}  // namespace subsea_interfaces


namespace subsea_interfaces
{

namespace srv
{

namespace builder
{

class Init_CapturePair_Event_response
{
public:
  explicit Init_CapturePair_Event_response(::subsea_interfaces::srv::CapturePair_Event & msg)
  : msg_(msg)
  {}
  ::subsea_interfaces::srv::CapturePair_Event response(::subsea_interfaces::srv::CapturePair_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Event msg_;
};

class Init_CapturePair_Event_request
{
public:
  explicit Init_CapturePair_Event_request(::subsea_interfaces::srv::CapturePair_Event & msg)
  : msg_(msg)
  {}
  Init_CapturePair_Event_response request(::subsea_interfaces::srv::CapturePair_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CapturePair_Event_response(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Event msg_;
};

class Init_CapturePair_Event_info
{
public:
  Init_CapturePair_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CapturePair_Event_request info(::subsea_interfaces::srv::CapturePair_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CapturePair_Event_request(msg_);
  }

private:
  ::subsea_interfaces::srv::CapturePair_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::subsea_interfaces::srv::CapturePair_Event>()
{
  return subsea_interfaces::srv::builder::Init_CapturePair_Event_info();
}

}  // namespace subsea_interfaces

#endif  // SUBSEA_INTERFACES__SRV__DETAIL__CAPTURE_PAIR__BUILDER_HPP_
