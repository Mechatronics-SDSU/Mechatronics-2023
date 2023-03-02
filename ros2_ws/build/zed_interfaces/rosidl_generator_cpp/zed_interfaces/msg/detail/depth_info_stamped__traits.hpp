// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_interfaces:msg/DepthInfoStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__TRAITS_HPP_
#define ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__TRAITS_HPP_

#include "zed_interfaces/msg/detail/depth_info_stamped__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zed_interfaces::msg::DepthInfoStamped>()
{
  return "zed_interfaces::msg::DepthInfoStamped";
}

template<>
inline const char * name<zed_interfaces::msg::DepthInfoStamped>()
{
  return "zed_interfaces/msg/DepthInfoStamped";
}

template<>
struct has_fixed_size<zed_interfaces::msg::DepthInfoStamped>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<zed_interfaces::msg::DepthInfoStamped>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<zed_interfaces::msg::DepthInfoStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__TRAITS_HPP_
