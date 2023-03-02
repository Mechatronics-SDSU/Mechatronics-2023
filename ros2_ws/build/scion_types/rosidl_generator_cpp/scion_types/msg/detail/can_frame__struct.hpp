// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scion_types:msg/CanFrame.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__CAN_FRAME__STRUCT_HPP_
#define SCION_TYPES__MSG__DETAIL__CAN_FRAME__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__scion_types__msg__CanFrame __attribute__((deprecated))
#else
# define DEPRECATED__scion_types__msg__CanFrame __declspec(deprecated)
#endif

namespace scion_types
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CanFrame_
{
  using Type = CanFrame_<ContainerAllocator>;

  explicit CanFrame_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0l;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->can_data.begin(), this->can_data.end(), 0);
    }
  }

  explicit CanFrame_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : can_data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0l;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->can_data.begin(), this->can_data.end(), 0);
    }
  }

  // field types and members
  using _can_id_type =
    int32_t;
  _can_id_type can_id;
  using _can_data_type =
    std::array<uint8_t, 8>;
  _can_data_type can_data;

  // setters for named parameter idiom
  Type & set__can_id(
    const int32_t & _arg)
  {
    this->can_id = _arg;
    return *this;
  }
  Type & set__can_data(
    const std::array<uint8_t, 8> & _arg)
  {
    this->can_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scion_types::msg::CanFrame_<ContainerAllocator> *;
  using ConstRawPtr =
    const scion_types::msg::CanFrame_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scion_types::msg::CanFrame_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scion_types::msg::CanFrame_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scion_types::msg::CanFrame_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scion_types::msg::CanFrame_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scion_types::msg::CanFrame_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scion_types::msg::CanFrame_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scion_types::msg::CanFrame_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scion_types::msg::CanFrame_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scion_types__msg__CanFrame
    std::shared_ptr<scion_types::msg::CanFrame_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scion_types__msg__CanFrame
    std::shared_ptr<scion_types::msg::CanFrame_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CanFrame_ & other) const
  {
    if (this->can_id != other.can_id) {
      return false;
    }
    if (this->can_data != other.can_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const CanFrame_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CanFrame_

// alias to use template instance with default allocator
using CanFrame =
  scion_types::msg::CanFrame_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace scion_types

#endif  // SCION_TYPES__MSG__DETAIL__CAN_FRAME__STRUCT_HPP_
