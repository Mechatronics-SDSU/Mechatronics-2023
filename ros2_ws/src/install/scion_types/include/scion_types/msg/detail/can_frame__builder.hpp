// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scion_types:msg/CanFrame.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__CAN_FRAME__BUILDER_HPP_
#define SCION_TYPES__MSG__DETAIL__CAN_FRAME__BUILDER_HPP_

#include "scion_types/msg/detail/can_frame__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace scion_types
{

namespace msg
{

namespace builder
{

class Init_CanFrame_can_data
{
public:
  explicit Init_CanFrame_can_data(::scion_types::msg::CanFrame & msg)
  : msg_(msg)
  {}
  ::scion_types::msg::CanFrame can_data(::scion_types::msg::CanFrame::_can_data_type arg)
  {
    msg_.can_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scion_types::msg::CanFrame msg_;
};

class Init_CanFrame_can_id
{
public:
  Init_CanFrame_can_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CanFrame_can_data can_id(::scion_types::msg::CanFrame::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_CanFrame_can_data(msg_);
  }

private:
  ::scion_types::msg::CanFrame msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scion_types::msg::CanFrame>()
{
  return scion_types::msg::builder::Init_CanFrame_can_id();
}

}  // namespace scion_types

#endif  // SCION_TYPES__MSG__DETAIL__CAN_FRAME__BUILDER_HPP_
