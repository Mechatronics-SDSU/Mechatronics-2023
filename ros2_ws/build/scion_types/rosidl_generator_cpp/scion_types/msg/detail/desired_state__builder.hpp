// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scion_types:msg/DesiredState.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__DESIRED_STATE__BUILDER_HPP_
#define SCION_TYPES__MSG__DETAIL__DESIRED_STATE__BUILDER_HPP_

#include "scion_types/msg/detail/desired_state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace scion_types
{

namespace msg
{

namespace builder
{

class Init_DesiredState_desired_state
{
public:
  Init_DesiredState_desired_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::scion_types::msg::DesiredState desired_state(::scion_types::msg::DesiredState::_desired_state_type arg)
  {
    msg_.desired_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scion_types::msg::DesiredState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scion_types::msg::DesiredState>()
{
  return scion_types::msg::builder::Init_DesiredState_desired_state();
}

}  // namespace scion_types

#endif  // SCION_TYPES__MSG__DETAIL__DESIRED_STATE__BUILDER_HPP_
