// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:msg/DepthInfoStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__BUILDER_HPP_
#define ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__BUILDER_HPP_

#include "zed_interfaces/msg/detail/depth_info_stamped__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace zed_interfaces
{

namespace msg
{

namespace builder
{

class Init_DepthInfoStamped_max_depth
{
public:
  explicit Init_DepthInfoStamped_max_depth(::zed_interfaces::msg::DepthInfoStamped & msg)
  : msg_(msg)
  {}
  ::zed_interfaces::msg::DepthInfoStamped max_depth(::zed_interfaces::msg::DepthInfoStamped::_max_depth_type arg)
  {
    msg_.max_depth = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::msg::DepthInfoStamped msg_;
};

class Init_DepthInfoStamped_min_depth
{
public:
  explicit Init_DepthInfoStamped_min_depth(::zed_interfaces::msg::DepthInfoStamped & msg)
  : msg_(msg)
  {}
  Init_DepthInfoStamped_max_depth min_depth(::zed_interfaces::msg::DepthInfoStamped::_min_depth_type arg)
  {
    msg_.min_depth = std::move(arg);
    return Init_DepthInfoStamped_max_depth(msg_);
  }

private:
  ::zed_interfaces::msg::DepthInfoStamped msg_;
};

class Init_DepthInfoStamped_header
{
public:
  Init_DepthInfoStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DepthInfoStamped_min_depth header(::zed_interfaces::msg::DepthInfoStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DepthInfoStamped_min_depth(msg_);
  }

private:
  ::zed_interfaces::msg::DepthInfoStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::msg::DepthInfoStamped>()
{
  return zed_interfaces::msg::builder::Init_DepthInfoStamped_header();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__BUILDER_HPP_
