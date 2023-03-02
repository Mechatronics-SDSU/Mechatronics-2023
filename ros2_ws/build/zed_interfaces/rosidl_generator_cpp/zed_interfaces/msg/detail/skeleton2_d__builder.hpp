// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:msg/Skeleton2D.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__SKELETON2_D__BUILDER_HPP_
#define ZED_INTERFACES__MSG__DETAIL__SKELETON2_D__BUILDER_HPP_

#include "zed_interfaces/msg/detail/skeleton2_d__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace zed_interfaces
{

namespace msg
{

namespace builder
{

class Init_Skeleton2D_keypoints
{
public:
  Init_Skeleton2D_keypoints()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::zed_interfaces::msg::Skeleton2D keypoints(::zed_interfaces::msg::Skeleton2D::_keypoints_type arg)
  {
    msg_.keypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::msg::Skeleton2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::msg::Skeleton2D>()
{
  return zed_interfaces::msg::builder::Init_Skeleton2D_keypoints();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__SKELETON2_D__BUILDER_HPP_
