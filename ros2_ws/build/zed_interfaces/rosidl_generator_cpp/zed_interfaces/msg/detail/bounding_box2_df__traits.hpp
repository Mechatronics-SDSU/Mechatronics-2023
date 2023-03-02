// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_interfaces:msg/BoundingBox2Df.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DF__TRAITS_HPP_
#define ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DF__TRAITS_HPP_

#include "zed_interfaces/msg/detail/bounding_box2_df__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'corners'
#include "zed_interfaces/msg/detail/keypoint2_df__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zed_interfaces::msg::BoundingBox2Df>()
{
  return "zed_interfaces::msg::BoundingBox2Df";
}

template<>
inline const char * name<zed_interfaces::msg::BoundingBox2Df>()
{
  return "zed_interfaces/msg/BoundingBox2Df";
}

template<>
struct has_fixed_size<zed_interfaces::msg::BoundingBox2Df>
  : std::integral_constant<bool, has_fixed_size<zed_interfaces::msg::Keypoint2Df>::value> {};

template<>
struct has_bounded_size<zed_interfaces::msg::BoundingBox2Df>
  : std::integral_constant<bool, has_bounded_size<zed_interfaces::msg::Keypoint2Df>::value> {};

template<>
struct is_message<zed_interfaces::msg::BoundingBox2Df>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DF__TRAITS_HPP_
