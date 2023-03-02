// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:msg/ObjectsStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__OBJECTS_STAMPED__BUILDER_HPP_
#define ZED_INTERFACES__MSG__DETAIL__OBJECTS_STAMPED__BUILDER_HPP_

#include "zed_interfaces/msg/detail/objects_stamped__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace zed_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectsStamped_objects
{
public:
  explicit Init_ObjectsStamped_objects(::zed_interfaces::msg::ObjectsStamped & msg)
  : msg_(msg)
  {}
  ::zed_interfaces::msg::ObjectsStamped objects(::zed_interfaces::msg::ObjectsStamped::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::msg::ObjectsStamped msg_;
};

class Init_ObjectsStamped_header
{
public:
  Init_ObjectsStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectsStamped_objects header(::zed_interfaces::msg::ObjectsStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectsStamped_objects(msg_);
  }

private:
  ::zed_interfaces::msg::ObjectsStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::msg::ObjectsStamped>()
{
  return zed_interfaces::msg::builder::Init_ObjectsStamped_header();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__OBJECTS_STAMPED__BUILDER_HPP_
