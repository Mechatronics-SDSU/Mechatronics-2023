// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_interfaces:msg/ObjectsStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__OBJECTS_STAMPED__TRAITS_HPP_
#define ZED_INTERFACES__MSG__DETAIL__OBJECTS_STAMPED__TRAITS_HPP_

#include "zed_interfaces/msg/detail/objects_stamped__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zed_interfaces::msg::ObjectsStamped>()
{
  return "zed_interfaces::msg::ObjectsStamped";
}

template<>
inline const char * name<zed_interfaces::msg::ObjectsStamped>()
{
  return "zed_interfaces/msg/ObjectsStamped";
}

template<>
struct has_fixed_size<zed_interfaces::msg::ObjectsStamped>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<zed_interfaces::msg::ObjectsStamped>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<zed_interfaces::msg::ObjectsStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZED_INTERFACES__MSG__DETAIL__OBJECTS_STAMPED__TRAITS_HPP_
