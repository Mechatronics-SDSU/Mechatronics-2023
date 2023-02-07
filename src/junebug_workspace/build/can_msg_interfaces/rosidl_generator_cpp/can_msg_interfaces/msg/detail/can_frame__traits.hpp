// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from can_msg_interfaces:msg/CanFrame.idl
// generated code does not contain a copyright notice

#ifndef CAN_MSG_INTERFACES__MSG__DETAIL__CAN_FRAME__TRAITS_HPP_
#define CAN_MSG_INTERFACES__MSG__DETAIL__CAN_FRAME__TRAITS_HPP_

#include "can_msg_interfaces/msg/detail/can_frame__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<can_msg_interfaces::msg::CanFrame>()
{
  return "can_msg_interfaces::msg::CanFrame";
}

template<>
inline const char * name<can_msg_interfaces::msg::CanFrame>()
{
  return "can_msg_interfaces/msg/CanFrame";
}

template<>
struct has_fixed_size<can_msg_interfaces::msg::CanFrame>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<can_msg_interfaces::msg::CanFrame>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<can_msg_interfaces::msg::CanFrame>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CAN_MSG_INTERFACES__MSG__DETAIL__CAN_FRAME__TRAITS_HPP_
