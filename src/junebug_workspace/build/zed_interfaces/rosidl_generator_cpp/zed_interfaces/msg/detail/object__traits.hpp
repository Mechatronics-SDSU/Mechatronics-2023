// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__OBJECT__TRAITS_HPP_
#define ZED_INTERFACES__MSG__DETAIL__OBJECT__TRAITS_HPP_

#include "zed_interfaces/msg/detail/object__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'bounding_box_2d'
#include "zed_interfaces/msg/detail/bounding_box2_di__traits.hpp"
// Member 'bounding_box_3d'
// Member 'head_bounding_box_3d'
#include "zed_interfaces/msg/detail/bounding_box3_d__traits.hpp"
// Member 'head_bounding_box_2d'
#include "zed_interfaces/msg/detail/bounding_box2_df__traits.hpp"
// Member 'skeleton_2d'
#include "zed_interfaces/msg/detail/skeleton2_d__traits.hpp"
// Member 'skeleton_3d'
#include "zed_interfaces/msg/detail/skeleton3_d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zed_interfaces::msg::Object>()
{
  return "zed_interfaces::msg::Object";
}

template<>
inline const char * name<zed_interfaces::msg::Object>()
{
  return "zed_interfaces/msg/Object";
}

template<>
struct has_fixed_size<zed_interfaces::msg::Object>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<zed_interfaces::msg::Object>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<zed_interfaces::msg::Object>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZED_INTERFACES__MSG__DETAIL__OBJECT__TRAITS_HPP_
