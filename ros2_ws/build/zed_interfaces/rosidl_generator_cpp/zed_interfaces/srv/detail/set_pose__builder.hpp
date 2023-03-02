// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:srv/SetPose.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__SRV__DETAIL__SET_POSE__BUILDER_HPP_
#define ZED_INTERFACES__SRV__DETAIL__SET_POSE__BUILDER_HPP_

#include "zed_interfaces/srv/detail/set_pose__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace zed_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetPose_Request_orient
{
public:
  explicit Init_SetPose_Request_orient(::zed_interfaces::srv::SetPose_Request & msg)
  : msg_(msg)
  {}
  ::zed_interfaces::srv::SetPose_Request orient(::zed_interfaces::srv::SetPose_Request::_orient_type arg)
  {
    msg_.orient = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::srv::SetPose_Request msg_;
};

class Init_SetPose_Request_pos
{
public:
  Init_SetPose_Request_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPose_Request_orient pos(::zed_interfaces::srv::SetPose_Request::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_SetPose_Request_orient(msg_);
  }

private:
  ::zed_interfaces::srv::SetPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::srv::SetPose_Request>()
{
  return zed_interfaces::srv::builder::Init_SetPose_Request_pos();
}

}  // namespace zed_interfaces


namespace zed_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetPose_Response_message
{
public:
  explicit Init_SetPose_Response_message(::zed_interfaces::srv::SetPose_Response & msg)
  : msg_(msg)
  {}
  ::zed_interfaces::srv::SetPose_Response message(::zed_interfaces::srv::SetPose_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::srv::SetPose_Response msg_;
};

class Init_SetPose_Response_success
{
public:
  Init_SetPose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPose_Response_message success(::zed_interfaces::srv::SetPose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetPose_Response_message(msg_);
  }

private:
  ::zed_interfaces::srv::SetPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::srv::SetPose_Response>()
{
  return zed_interfaces::srv::builder::Init_SetPose_Response_success();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__SRV__DETAIL__SET_POSE__BUILDER_HPP_
