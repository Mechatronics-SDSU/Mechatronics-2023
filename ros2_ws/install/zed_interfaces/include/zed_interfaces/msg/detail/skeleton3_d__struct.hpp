// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:msg/Skeleton3D.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__SKELETON3_D__STRUCT_HPP_
#define ZED_INTERFACES__MSG__DETAIL__SKELETON3_D__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'keypoints'
#include "zed_interfaces/msg/detail/keypoint3_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__zed_interfaces__msg__Skeleton3D __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__msg__Skeleton3D __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Skeleton3D_
{
  using Type = Skeleton3D_<ContainerAllocator>;

  explicit Skeleton3D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->keypoints.fill(zed_interfaces::msg::Keypoint3D_<ContainerAllocator>{_init});
    }
  }

  explicit Skeleton3D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : keypoints(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->keypoints.fill(zed_interfaces::msg::Keypoint3D_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _keypoints_type =
    std::array<zed_interfaces::msg::Keypoint3D_<ContainerAllocator>, 34>;
  _keypoints_type keypoints;

  // setters for named parameter idiom
  Type & set__keypoints(
    const std::array<zed_interfaces::msg::Keypoint3D_<ContainerAllocator>, 34> & _arg)
  {
    this->keypoints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::msg::Skeleton3D_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::msg::Skeleton3D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::msg::Skeleton3D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::msg::Skeleton3D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::Skeleton3D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::Skeleton3D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::Skeleton3D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::Skeleton3D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::msg::Skeleton3D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::msg::Skeleton3D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__msg__Skeleton3D
    std::shared_ptr<zed_interfaces::msg::Skeleton3D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__msg__Skeleton3D
    std::shared_ptr<zed_interfaces::msg::Skeleton3D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Skeleton3D_ & other) const
  {
    if (this->keypoints != other.keypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const Skeleton3D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Skeleton3D_

// alias to use template instance with default allocator
using Skeleton3D =
  zed_interfaces::msg::Skeleton3D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__SKELETON3_D__STRUCT_HPP_
