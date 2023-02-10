// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scion_types:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__POSITION__BUILDER_HPP_
#define SCION_TYPES__MSG__DETAIL__POSITION__BUILDER_HPP_

#include "scion_types/msg/detail/position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace scion_types
{

namespace msg
{

namespace builder
{

class Init_Position_position
{
public:
  Init_Position_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::scion_types::msg::Position position(::scion_types::msg::Position::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scion_types::msg::Position msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scion_types::msg::Position>()
{
  return scion_types::msg::builder::Init_Position_position();
}

}  // namespace scion_types

#endif  // SCION_TYPES__MSG__DETAIL__POSITION__BUILDER_HPP_
