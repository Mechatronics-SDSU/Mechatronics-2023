// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:msg/ObjectsStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__OBJECTS_STAMPED__STRUCT_HPP_
#define ZED_INTERFACES__MSG__DETAIL__OBJECTS_STAMPED__STRUCT_HPP_

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
// Member 'objects'
#include "zed_interfaces/msg/detail/object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__zed_interfaces__msg__ObjectsStamped __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__msg__ObjectsStamped __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectsStamped_
{
  using Type = ObjectsStamped_<ContainerAllocator>;

  explicit ObjectsStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ObjectsStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _objects_type =
    std::vector<zed_interfaces::msg::Object_<ContainerAllocator>, typename ContainerAllocator::template rebind<zed_interfaces::msg::Object_<ContainerAllocator>>::other>;
  _objects_type objects;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__objects(
    const std::vector<zed_interfaces::msg::Object_<ContainerAllocator>, typename ContainerAllocator::template rebind<zed_interfaces::msg::Object_<ContainerAllocator>>::other> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::msg::ObjectsStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::msg::ObjectsStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::msg::ObjectsStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::msg::ObjectsStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::ObjectsStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::ObjectsStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::ObjectsStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::ObjectsStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::msg::ObjectsStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::msg::ObjectsStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__msg__ObjectsStamped
    std::shared_ptr<zed_interfaces::msg::ObjectsStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__msg__ObjectsStamped
    std::shared_ptr<zed_interfaces::msg::ObjectsStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectsStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectsStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectsStamped_

// alias to use template instance with default allocator
using ObjectsStamped =
  zed_interfaces::msg::ObjectsStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__OBJECTS_STAMPED__STRUCT_HPP_
