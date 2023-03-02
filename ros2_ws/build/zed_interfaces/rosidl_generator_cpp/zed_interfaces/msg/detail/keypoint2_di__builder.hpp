// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:msg/Keypoint2Di.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__KEYPOINT2_DI__BUILDER_HPP_
#define ZED_INTERFACES__MSG__DETAIL__KEYPOINT2_DI__BUILDER_HPP_

#include "zed_interfaces/msg/detail/keypoint2_di__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace zed_interfaces
{

namespace msg
{

namespace builder
{

class Init_Keypoint2Di_kp
{
public:
  Init_Keypoint2Di_kp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::zed_interfaces::msg::Keypoint2Di kp(::zed_interfaces::msg::Keypoint2Di::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::msg::Keypoint2Di msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::msg::Keypoint2Di>()
{
  return zed_interfaces::msg::builder::Init_Keypoint2Di_kp();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__KEYPOINT2_DI__BUILDER_HPP_
