// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:srv/StartSvoRec.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__SRV__DETAIL__START_SVO_REC__BUILDER_HPP_
#define ZED_INTERFACES__SRV__DETAIL__START_SVO_REC__BUILDER_HPP_

#include "zed_interfaces/srv/detail/start_svo_rec__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace zed_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartSvoRec_Request_svo_filename
{
public:
  explicit Init_StartSvoRec_Request_svo_filename(::zed_interfaces::srv::StartSvoRec_Request & msg)
  : msg_(msg)
  {}
  ::zed_interfaces::srv::StartSvoRec_Request svo_filename(::zed_interfaces::srv::StartSvoRec_Request::_svo_filename_type arg)
  {
    msg_.svo_filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::srv::StartSvoRec_Request msg_;
};

class Init_StartSvoRec_Request_input_transcode
{
public:
  explicit Init_StartSvoRec_Request_input_transcode(::zed_interfaces::srv::StartSvoRec_Request & msg)
  : msg_(msg)
  {}
  Init_StartSvoRec_Request_svo_filename input_transcode(::zed_interfaces::srv::StartSvoRec_Request::_input_transcode_type arg)
  {
    msg_.input_transcode = std::move(arg);
    return Init_StartSvoRec_Request_svo_filename(msg_);
  }

private:
  ::zed_interfaces::srv::StartSvoRec_Request msg_;
};

class Init_StartSvoRec_Request_target_framerate
{
public:
  explicit Init_StartSvoRec_Request_target_framerate(::zed_interfaces::srv::StartSvoRec_Request & msg)
  : msg_(msg)
  {}
  Init_StartSvoRec_Request_input_transcode target_framerate(::zed_interfaces::srv::StartSvoRec_Request::_target_framerate_type arg)
  {
    msg_.target_framerate = std::move(arg);
    return Init_StartSvoRec_Request_input_transcode(msg_);
  }

private:
  ::zed_interfaces::srv::StartSvoRec_Request msg_;
};

class Init_StartSvoRec_Request_compression_mode
{
public:
  explicit Init_StartSvoRec_Request_compression_mode(::zed_interfaces::srv::StartSvoRec_Request & msg)
  : msg_(msg)
  {}
  Init_StartSvoRec_Request_target_framerate compression_mode(::zed_interfaces::srv::StartSvoRec_Request::_compression_mode_type arg)
  {
    msg_.compression_mode = std::move(arg);
    return Init_StartSvoRec_Request_target_framerate(msg_);
  }

private:
  ::zed_interfaces::srv::StartSvoRec_Request msg_;
};

class Init_StartSvoRec_Request_bitrate
{
public:
  Init_StartSvoRec_Request_bitrate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartSvoRec_Request_compression_mode bitrate(::zed_interfaces::srv::StartSvoRec_Request::_bitrate_type arg)
  {
    msg_.bitrate = std::move(arg);
    return Init_StartSvoRec_Request_compression_mode(msg_);
  }

private:
  ::zed_interfaces::srv::StartSvoRec_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::srv::StartSvoRec_Request>()
{
  return zed_interfaces::srv::builder::Init_StartSvoRec_Request_bitrate();
}

}  // namespace zed_interfaces


namespace zed_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartSvoRec_Response_message
{
public:
  explicit Init_StartSvoRec_Response_message(::zed_interfaces::srv::StartSvoRec_Response & msg)
  : msg_(msg)
  {}
  ::zed_interfaces::srv::StartSvoRec_Response message(::zed_interfaces::srv::StartSvoRec_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::srv::StartSvoRec_Response msg_;
};

class Init_StartSvoRec_Response_success
{
public:
  Init_StartSvoRec_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartSvoRec_Response_message success(::zed_interfaces::srv::StartSvoRec_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_StartSvoRec_Response_message(msg_);
  }

private:
  ::zed_interfaces::srv::StartSvoRec_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::srv::StartSvoRec_Response>()
{
  return zed_interfaces::srv::builder::Init_StartSvoRec_Response_success();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__SRV__DETAIL__START_SVO_REC__BUILDER_HPP_
