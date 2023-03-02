// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_interfaces:msg/Skeleton3D.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__SKELETON3_D__TRAITS_HPP_
#define ZED_INTERFACES__MSG__DETAIL__SKELETON3_D__TRAITS_HPP_

#include "zed_interfaces/msg/detail/skeleton3_d__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'keypoints'
#include "zed_interfaces/msg/detail/keypoint3_d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zed_interfaces::msg::Skeleton3D>()
{
  return "zed_interfaces::msg::Skeleton3D";
}

template<>
inline const char * name<zed_interfaces::msg::Skeleton3D>()
{
  return "zed_interfaces/msg/Skeleton3D";
}

template<>
struct has_fixed_size<zed_interfaces::msg::Skeleton3D>
  : std::integral_constant<bool, has_fixed_size<zed_interfaces::msg::Keypoint3D>::value> {};

template<>
struct has_bounded_size<zed_interfaces::msg::Skeleton3D>
  : std::integral_constant<bool, has_bounded_size<zed_interfaces::msg::Keypoint3D>::value> {};

template<>
struct is_message<zed_interfaces::msg::Skeleton3D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZED_INTERFACES__MSG__DETAIL__SKELETON3_D__TRAITS_HPP_
