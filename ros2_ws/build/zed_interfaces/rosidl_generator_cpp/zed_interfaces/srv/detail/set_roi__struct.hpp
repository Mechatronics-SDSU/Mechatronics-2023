// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:srv/SetROI.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__SRV__DETAIL__SET_ROI__STRUCT_HPP_
#define ZED_INTERFACES__SRV__DETAIL__SET_ROI__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__zed_interfaces__srv__SetROI_Request __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__srv__SetROI_Request __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetROI_Request_
{
  using Type = SetROI_Request_<ContainerAllocator>;

  explicit SetROI_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->roi = "";
    }
  }

  explicit SetROI_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : roi(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->roi = "";
    }
  }

  // field types and members
  using _roi_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _roi_type roi;

  // setters for named parameter idiom
  Type & set__roi(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->roi = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::srv::SetROI_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::srv::SetROI_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::srv::SetROI_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::srv::SetROI_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::srv::SetROI_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::srv::SetROI_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::srv::SetROI_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::srv::SetROI_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::srv::SetROI_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::srv::SetROI_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__srv__SetROI_Request
    std::shared_ptr<zed_interfaces::srv::SetROI_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__srv__SetROI_Request
    std::shared_ptr<zed_interfaces::srv::SetROI_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetROI_Request_ & other) const
  {
    if (this->roi != other.roi) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetROI_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetROI_Request_

// alias to use template instance with default allocator
using SetROI_Request =
  zed_interfaces::srv::SetROI_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace zed_interfaces


#ifndef _WIN32
# define DEPRECATED__zed_interfaces__srv__SetROI_Response __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__srv__SetROI_Response __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetROI_Response_
{
  using Type = SetROI_Response_<ContainerAllocator>;

  explicit SetROI_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetROI_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::srv::SetROI_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::srv::SetROI_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::srv::SetROI_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::srv::SetROI_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::srv::SetROI_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::srv::SetROI_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::srv::SetROI_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::srv::SetROI_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::srv::SetROI_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::srv::SetROI_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__srv__SetROI_Response
    std::shared_ptr<zed_interfaces::srv::SetROI_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__srv__SetROI_Response
    std::shared_ptr<zed_interfaces::srv::SetROI_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetROI_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetROI_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetROI_Response_

// alias to use template instance with default allocator
using SetROI_Response =
  zed_interfaces::srv::SetROI_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace zed_interfaces

namespace zed_interfaces
{

namespace srv
{

struct SetROI
{
  using Request = zed_interfaces::srv::SetROI_Request;
  using Response = zed_interfaces::srv::SetROI_Response;
};

}  // namespace srv

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__SRV__DETAIL__SET_ROI__STRUCT_HPP_
