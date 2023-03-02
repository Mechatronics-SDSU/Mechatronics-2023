// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scion_types:msg/DesiredState.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__DESIRED_STATE__STRUCT_HPP_
#define SCION_TYPES__MSG__DETAIL__DESIRED_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__scion_types__msg__DesiredState __attribute__((deprecated))
#else
# define DEPRECATED__scion_types__msg__DesiredState __declspec(deprecated)
#endif

namespace scion_types
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DesiredState_
{
  using Type = DesiredState_<ContainerAllocator>;

  explicit DesiredState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit DesiredState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _desired_state_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _desired_state_type desired_state;

  // setters for named parameter idiom
  Type & set__desired_state(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->desired_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scion_types::msg::DesiredState_<ContainerAllocator> *;
  using ConstRawPtr =
    const scion_types::msg::DesiredState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scion_types::msg::DesiredState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scion_types::msg::DesiredState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scion_types::msg::DesiredState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scion_types::msg::DesiredState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scion_types::msg::DesiredState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scion_types::msg::DesiredState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scion_types::msg::DesiredState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scion_types::msg::DesiredState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scion_types__msg__DesiredState
    std::shared_ptr<scion_types::msg::DesiredState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scion_types__msg__DesiredState
    std::shared_ptr<scion_types::msg::DesiredState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DesiredState_ & other) const
  {
    if (this->desired_state != other.desired_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const DesiredState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DesiredState_

// alias to use template instance with default allocator
using DesiredState =
  scion_types::msg::DesiredState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace scion_types

#endif  // SCION_TYPES__MSG__DETAIL__DESIRED_STATE__STRUCT_HPP_
