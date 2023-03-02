// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:msg/BoundingBox2Di.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DI__BUILDER_HPP_
#define ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DI__BUILDER_HPP_

#include "zed_interfaces/msg/detail/bounding_box2_di__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace zed_interfaces
{

namespace msg
{

namespace builder
{

class Init_BoundingBox2Di_corners
{
public:
  Init_BoundingBox2Di_corners()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::zed_interfaces::msg::BoundingBox2Di corners(::zed_interfaces::msg::BoundingBox2Di::_corners_type arg)
  {
    msg_.corners = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::msg::BoundingBox2Di msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::msg::BoundingBox2Di>()
{
  return zed_interfaces::msg::builder::Init_BoundingBox2Di_corners();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DI__BUILDER_HPP_
