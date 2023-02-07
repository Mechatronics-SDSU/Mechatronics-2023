// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:msg/BoundingBox3D.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX3_D__BUILDER_HPP_
#define ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX3_D__BUILDER_HPP_

#include "zed_interfaces/msg/detail/bounding_box3_d__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace zed_interfaces
{

namespace msg
{

namespace builder
{

class Init_BoundingBox3D_corners
{
public:
  Init_BoundingBox3D_corners()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::zed_interfaces::msg::BoundingBox3D corners(::zed_interfaces::msg::BoundingBox3D::_corners_type arg)
  {
    msg_.corners = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::msg::BoundingBox3D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::msg::BoundingBox3D>()
{
  return zed_interfaces::msg::builder::Init_BoundingBox3D_corners();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX3_D__BUILDER_HPP_
