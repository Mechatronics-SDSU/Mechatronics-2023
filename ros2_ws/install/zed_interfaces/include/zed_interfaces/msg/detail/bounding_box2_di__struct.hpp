// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:msg/BoundingBox2Di.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DI__STRUCT_HPP_
#define ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DI__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'corners'
#include "zed_interfaces/msg/detail/keypoint2_di__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__zed_interfaces__msg__BoundingBox2Di __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__msg__BoundingBox2Di __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox2Di_
{
  using Type = BoundingBox2Di_<ContainerAllocator>;

  explicit BoundingBox2Di_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->corners.fill(zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>{_init});
    }
  }

  explicit BoundingBox2Di_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : corners(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->corners.fill(zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _corners_type =
    std::array<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>, 4>;
  _corners_type corners;

  // setters for named parameter idiom
  Type & set__corners(
    const std::array<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>, 4> & _arg)
  {
    this->corners = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__msg__BoundingBox2Di
    std::shared_ptr<zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__msg__BoundingBox2Di
    std::shared_ptr<zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox2Di_ & other) const
  {
    if (this->corners != other.corners) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox2Di_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox2Di_

// alias to use template instance with default allocator
using BoundingBox2Di =
  zed_interfaces::msg::BoundingBox2Di_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DI__STRUCT_HPP_
