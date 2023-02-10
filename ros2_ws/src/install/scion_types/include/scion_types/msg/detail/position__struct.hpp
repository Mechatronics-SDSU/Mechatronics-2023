// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scion_types:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__POSITION__STRUCT_HPP_
#define SCION_TYPES__MSG__DETAIL__POSITION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__scion_types__msg__Position __attribute__((deprecated))
#else
# define DEPRECATED__scion_types__msg__Position __declspec(deprecated)
#endif

namespace scion_types
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Position_
{
  using Type = Position_<ContainerAllocator>;

  explicit Position_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Position_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _position_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _position_type position;

  // setters for named parameter idiom
  Type & set__position(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scion_types::msg::Position_<ContainerAllocator> *;
  using ConstRawPtr =
    const scion_types::msg::Position_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scion_types::msg::Position_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scion_types::msg::Position_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scion_types::msg::Position_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scion_types::msg::Position_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scion_types::msg::Position_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scion_types::msg::Position_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scion_types::msg::Position_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scion_types::msg::Position_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scion_types__msg__Position
    std::shared_ptr<scion_types::msg::Position_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scion_types__msg__Position
    std::shared_ptr<scion_types::msg::Position_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Position_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const Position_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Position_

// alias to use template instance with default allocator
using Position =
  scion_types::msg::Position_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace scion_types

#endif  // SCION_TYPES__MSG__DETAIL__POSITION__STRUCT_HPP_
