// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zed_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__OBJECT__BUILDER_HPP_
#define ZED_INTERFACES__MSG__DETAIL__OBJECT__BUILDER_HPP_

#include "zed_interfaces/msg/detail/object__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace zed_interfaces
{

namespace msg
{

namespace builder
{

class Init_Object_skeleton_3d
{
public:
  explicit Init_Object_skeleton_3d(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  ::zed_interfaces::msg::Object skeleton_3d(::zed_interfaces::msg::Object::_skeleton_3d_type arg)
  {
    msg_.skeleton_3d = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_skeleton_2d
{
public:
  explicit Init_Object_skeleton_2d(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_skeleton_3d skeleton_2d(::zed_interfaces::msg::Object::_skeleton_2d_type arg)
  {
    msg_.skeleton_2d = std::move(arg);
    return Init_Object_skeleton_3d(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_head_position
{
public:
  explicit Init_Object_head_position(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_skeleton_2d head_position(::zed_interfaces::msg::Object::_head_position_type arg)
  {
    msg_.head_position = std::move(arg);
    return Init_Object_skeleton_2d(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_head_bounding_box_3d
{
public:
  explicit Init_Object_head_bounding_box_3d(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_head_position head_bounding_box_3d(::zed_interfaces::msg::Object::_head_bounding_box_3d_type arg)
  {
    msg_.head_bounding_box_3d = std::move(arg);
    return Init_Object_head_position(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_head_bounding_box_2d
{
public:
  explicit Init_Object_head_bounding_box_2d(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_head_bounding_box_3d head_bounding_box_2d(::zed_interfaces::msg::Object::_head_bounding_box_2d_type arg)
  {
    msg_.head_bounding_box_2d = std::move(arg);
    return Init_Object_head_bounding_box_3d(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_body_format
{
public:
  explicit Init_Object_body_format(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_head_bounding_box_2d body_format(::zed_interfaces::msg::Object::_body_format_type arg)
  {
    msg_.body_format = std::move(arg);
    return Init_Object_head_bounding_box_2d(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_skeleton_available
{
public:
  explicit Init_Object_skeleton_available(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_body_format skeleton_available(::zed_interfaces::msg::Object::_skeleton_available_type arg)
  {
    msg_.skeleton_available = std::move(arg);
    return Init_Object_body_format(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_dimensions_3d
{
public:
  explicit Init_Object_dimensions_3d(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_skeleton_available dimensions_3d(::zed_interfaces::msg::Object::_dimensions_3d_type arg)
  {
    msg_.dimensions_3d = std::move(arg);
    return Init_Object_skeleton_available(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_bounding_box_3d
{
public:
  explicit Init_Object_bounding_box_3d(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_dimensions_3d bounding_box_3d(::zed_interfaces::msg::Object::_bounding_box_3d_type arg)
  {
    msg_.bounding_box_3d = std::move(arg);
    return Init_Object_dimensions_3d(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_bounding_box_2d
{
public:
  explicit Init_Object_bounding_box_2d(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_bounding_box_3d bounding_box_2d(::zed_interfaces::msg::Object::_bounding_box_2d_type arg)
  {
    msg_.bounding_box_2d = std::move(arg);
    return Init_Object_bounding_box_3d(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_action_state
{
public:
  explicit Init_Object_action_state(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_bounding_box_2d action_state(::zed_interfaces::msg::Object::_action_state_type arg)
  {
    msg_.action_state = std::move(arg);
    return Init_Object_bounding_box_2d(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_tracking_state
{
public:
  explicit Init_Object_tracking_state(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_action_state tracking_state(::zed_interfaces::msg::Object::_tracking_state_type arg)
  {
    msg_.tracking_state = std::move(arg);
    return Init_Object_action_state(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_tracking_available
{
public:
  explicit Init_Object_tracking_available(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_tracking_state tracking_available(::zed_interfaces::msg::Object::_tracking_available_type arg)
  {
    msg_.tracking_available = std::move(arg);
    return Init_Object_tracking_state(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_velocity
{
public:
  explicit Init_Object_velocity(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_tracking_available velocity(::zed_interfaces::msg::Object::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Object_tracking_available(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_position_covariance
{
public:
  explicit Init_Object_position_covariance(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_velocity position_covariance(::zed_interfaces::msg::Object::_position_covariance_type arg)
  {
    msg_.position_covariance = std::move(arg);
    return Init_Object_velocity(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_position
{
public:
  explicit Init_Object_position(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_position_covariance position(::zed_interfaces::msg::Object::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Object_position_covariance(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_confidence
{
public:
  explicit Init_Object_confidence(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_position confidence(::zed_interfaces::msg::Object::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Object_position(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_sublabel
{
public:
  explicit Init_Object_sublabel(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_confidence sublabel(::zed_interfaces::msg::Object::_sublabel_type arg)
  {
    msg_.sublabel = std::move(arg);
    return Init_Object_confidence(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_label_id
{
public:
  explicit Init_Object_label_id(::zed_interfaces::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_sublabel label_id(::zed_interfaces::msg::Object::_label_id_type arg)
  {
    msg_.label_id = std::move(arg);
    return Init_Object_sublabel(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

class Init_Object_label
{
public:
  Init_Object_label()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Object_label_id label(::zed_interfaces::msg::Object::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_Object_label_id(msg_);
  }

private:
  ::zed_interfaces::msg::Object msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zed_interfaces::msg::Object>()
{
  return zed_interfaces::msg::builder::Init_Object_label();
}

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__OBJECT__BUILDER_HPP_
