// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scion_types:srv/SendFrame.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__SRV__DETAIL__SEND_FRAME__BUILDER_HPP_
#define SCION_TYPES__SRV__DETAIL__SEND_FRAME__BUILDER_HPP_

#include "scion_types/srv/detail/send_frame__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace scion_types
{

namespace srv
{

namespace builder
{

class Init_SendFrame_Request_can_data
{
public:
  explicit Init_SendFrame_Request_can_data(::scion_types::srv::SendFrame_Request & msg)
  : msg_(msg)
  {}
  ::scion_types::srv::SendFrame_Request can_data(::scion_types::srv::SendFrame_Request::_can_data_type arg)
  {
    msg_.can_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scion_types::srv::SendFrame_Request msg_;
};

class Init_SendFrame_Request_can_dlc
{
public:
  explicit Init_SendFrame_Request_can_dlc(::scion_types::srv::SendFrame_Request & msg)
  : msg_(msg)
  {}
  Init_SendFrame_Request_can_data can_dlc(::scion_types::srv::SendFrame_Request::_can_dlc_type arg)
  {
    msg_.can_dlc = std::move(arg);
    return Init_SendFrame_Request_can_data(msg_);
  }

private:
  ::scion_types::srv::SendFrame_Request msg_;
};

class Init_SendFrame_Request_can_id
{
public:
  Init_SendFrame_Request_can_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendFrame_Request_can_dlc can_id(::scion_types::srv::SendFrame_Request::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_SendFrame_Request_can_dlc(msg_);
  }

private:
  ::scion_types::srv::SendFrame_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::scion_types::srv::SendFrame_Request>()
{
  return scion_types::srv::builder::Init_SendFrame_Request_can_id();
}

}  // namespace scion_types


namespace scion_types
{

namespace srv
{

namespace builder
{

class Init_SendFrame_Response_status
{
public:
  Init_SendFrame_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::scion_types::srv::SendFrame_Response status(::scion_types::srv::SendFrame_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scion_types::srv::SendFrame_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::scion_types::srv::SendFrame_Response>()
{
  return scion_types::srv::builder::Init_SendFrame_Response_status();
}

}  // namespace scion_types

#endif  // SCION_TYPES__SRV__DETAIL__SEND_FRAME__BUILDER_HPP_
