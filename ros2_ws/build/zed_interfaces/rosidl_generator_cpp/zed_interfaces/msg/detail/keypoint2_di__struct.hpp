// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:msg/Keypoint2Di.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__KEYPOINT2_DI__STRUCT_HPP_
#define ZED_INTERFACES__MSG__DETAIL__KEYPOINT2_DI__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__zed_interfaces__msg__Keypoint2Di __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__msg__Keypoint2Di __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Keypoint2Di_
{
  using Type = Keypoint2Di_<ContainerAllocator>;

  explicit Keypoint2Di_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->kp.begin(), this->kp.end(), 0ul);
    }
  }

  explicit Keypoint2Di_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : kp(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint32_t, 2>::iterator, uint32_t>(this->kp.begin(), this->kp.end(), 0ul);
    }
  }

  // field types and members
  using _kp_type =
    std::array<uint32_t, 2>;
  _kp_type kp;

  // setters for named parameter idiom
  Type & set__kp(
    const std::array<uint32_t, 2> & _arg)
  {
    this->kp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::msg::Keypoint2Di_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::msg::Keypoint2Di_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__msg__Keypoint2Di
    std::shared_ptr<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__msg__Keypoint2Di
    std::shared_ptr<zed_interfaces::msg::Keypoint2Di_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Keypoint2Di_ & other) const
  {
    if (this->kp != other.kp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Keypoint2Di_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Keypoint2Di_

// alias to use template instance with default allocator
using Keypoint2Di =
  zed_interfaces::msg::Keypoint2Di_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__KEYPOINT2_DI__STRUCT_HPP_
