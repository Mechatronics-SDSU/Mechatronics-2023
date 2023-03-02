// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:msg/DepthInfoStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__STRUCT_HPP_
#define ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__zed_interfaces__msg__DepthInfoStamped __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__msg__DepthInfoStamped __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DepthInfoStamped_
{
  using Type = DepthInfoStamped_<ContainerAllocator>;

  explicit DepthInfoStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->min_depth = 0.0f;
      this->max_depth = 0.0f;
    }
  }

  explicit DepthInfoStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->min_depth = 0.0f;
      this->max_depth = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _min_depth_type =
    float;
  _min_depth_type min_depth;
  using _max_depth_type =
    float;
  _max_depth_type max_depth;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__min_depth(
    const float & _arg)
  {
    this->min_depth = _arg;
    return *this;
  }
  Type & set__max_depth(
    const float & _arg)
  {
    this->max_depth = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__msg__DepthInfoStamped
    std::shared_ptr<zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__msg__DepthInfoStamped
    std::shared_ptr<zed_interfaces::msg::DepthInfoStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DepthInfoStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->min_depth != other.min_depth) {
      return false;
    }
    if (this->max_depth != other.max_depth) {
      return false;
    }
    return true;
  }
  bool operator!=(const DepthInfoStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DepthInfoStamped_

// alias to use template instance with default allocator
using DepthInfoStamped =
  zed_interfaces::msg::DepthInfoStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__STRUCT_HPP_
