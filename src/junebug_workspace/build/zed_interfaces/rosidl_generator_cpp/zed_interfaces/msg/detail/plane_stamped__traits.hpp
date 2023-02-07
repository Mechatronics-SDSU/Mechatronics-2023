// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_interfaces:msg/PlaneStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__PLANE_STAMPED__TRAITS_HPP_
#define ZED_INTERFACES__MSG__DETAIL__PLANE_STAMPED__TRAITS_HPP_

#include "zed_interfaces/msg/detail/plane_stamped__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'mesh'
#include "shape_msgs/msg/detail/mesh__traits.hpp"
// Member 'coefficients'
#include "shape_msgs/msg/detail/plane__traits.hpp"
// Member 'normal'
// Member 'center'
#include "geometry_msgs/msg/detail/point32__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/transform__traits.hpp"
// Member 'bounds'
#include "geometry_msgs/msg/detail/polygon__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zed_interfaces::msg::PlaneStamped>()
{
  return "zed_interfaces::msg::PlaneStamped";
}

template<>
inline const char * name<zed_interfaces::msg::PlaneStamped>()
{
  return "zed_interfaces/msg/PlaneStamped";
}

template<>
struct has_fixed_size<zed_interfaces::msg::PlaneStamped>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point32>::value && has_fixed_size<geometry_msgs::msg::Polygon>::value && has_fixed_size<geometry_msgs::msg::Transform>::value && has_fixed_size<shape_msgs::msg::Mesh>::value && has_fixed_size<shape_msgs::msg::Plane>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<zed_interfaces::msg::PlaneStamped>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point32>::value && has_bounded_size<geometry_msgs::msg::Polygon>::value && has_bounded_size<geometry_msgs::msg::Transform>::value && has_bounded_size<shape_msgs::msg::Mesh>::value && has_bounded_size<shape_msgs::msg::Plane>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<zed_interfaces::msg::PlaneStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZED_INTERFACES__MSG__DETAIL__PLANE_STAMPED__TRAITS_HPP_
