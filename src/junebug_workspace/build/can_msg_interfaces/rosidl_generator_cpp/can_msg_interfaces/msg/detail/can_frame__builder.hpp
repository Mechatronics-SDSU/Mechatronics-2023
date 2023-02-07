// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from can_msg_interfaces:msg/CanFrame.idl
// generated code does not contain a copyright notice

#ifndef CAN_MSG_INTERFACES__MSG__DETAIL__CAN_FRAME__BUILDER_HPP_
#define CAN_MSG_INTERFACES__MSG__DETAIL__CAN_FRAME__BUILDER_HPP_

#include "can_msg_interfaces/msg/detail/can_frame__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace can_msg_interfaces
{

namespace msg
{

namespace builder
{

class Init_CanFrame_can_data
{
public:
  explicit Init_CanFrame_can_data(::can_msg_interfaces::msg::CanFrame & msg)
  : msg_(msg)
  {}
  ::can_msg_interfaces::msg::CanFrame can_data(::can_msg_interfaces::msg::CanFrame::_can_data_type arg)
  {
    msg_.can_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::can_msg_interfaces::msg::CanFrame msg_;
};

class Init_CanFrame_can_id
{
public:
  Init_CanFrame_can_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CanFrame_can_data can_id(::can_msg_interfaces::msg::CanFrame::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_CanFrame_can_data(msg_);
  }

private:
  ::can_msg_interfaces::msg::CanFrame msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::can_msg_interfaces::msg::CanFrame>()
{
  return can_msg_interfaces::msg::builder::Init_CanFrame_can_id();
}

}  // namespace can_msg_interfaces

#endif  // CAN_MSG_INTERFACES__MSG__DETAIL__CAN_FRAME__BUILDER_HPP_
