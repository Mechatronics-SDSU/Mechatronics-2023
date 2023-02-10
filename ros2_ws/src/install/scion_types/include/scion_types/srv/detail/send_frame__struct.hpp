// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scion_types:srv/SendFrame.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__SRV__DETAIL__SEND_FRAME__STRUCT_HPP_
#define SCION_TYPES__SRV__DETAIL__SEND_FRAME__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__scion_types__srv__SendFrame_Request __attribute__((deprecated))
#else
# define DEPRECATED__scion_types__srv__SendFrame_Request __declspec(deprecated)
#endif

namespace scion_types
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendFrame_Request_
{
  using Type = SendFrame_Request_<ContainerAllocator>;

  explicit SendFrame_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0l;
      this->can_dlc = 0;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->can_data.begin(), this->can_data.end(), 0);
    }
  }

  explicit SendFrame_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : can_data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0l;
      this->can_dlc = 0;
      std::fill<typename std::array<uint8_t, 8>::iterator, uint8_t>(this->can_data.begin(), this->can_data.end(), 0);
    }
  }

  // field types and members
  using _can_id_type =
    int32_t;
  _can_id_type can_id;
  using _can_dlc_type =
    int8_t;
  _can_dlc_type can_dlc;
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
  Type & set__can_dlc(
    const int8_t & _arg)
  {
    this->can_dlc = _arg;
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
    scion_types::srv::SendFrame_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const scion_types::srv::SendFrame_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scion_types::srv::SendFrame_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scion_types::srv::SendFrame_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scion_types::srv::SendFrame_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scion_types::srv::SendFrame_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scion_types::srv::SendFrame_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scion_types::srv::SendFrame_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scion_types::srv::SendFrame_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scion_types::srv::SendFrame_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scion_types__srv__SendFrame_Request
    std::shared_ptr<scion_types::srv::SendFrame_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scion_types__srv__SendFrame_Request
    std::shared_ptr<scion_types::srv::SendFrame_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendFrame_Request_ & other) const
  {
    if (this->can_id != other.can_id) {
      return false;
    }
    if (this->can_dlc != other.can_dlc) {
      return false;
    }
    if (this->can_data != other.can_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendFrame_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendFrame_Request_

// alias to use template instance with default allocator
using SendFrame_Request =
  scion_types::srv::SendFrame_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace scion_types


#ifndef _WIN32
# define DEPRECATED__scion_types__srv__SendFrame_Response __attribute__((deprecated))
#else
# define DEPRECATED__scion_types__srv__SendFrame_Response __declspec(deprecated)
#endif

namespace scion_types
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SendFrame_Response_
{
  using Type = SendFrame_Response_<ContainerAllocator>;

  explicit SendFrame_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit SendFrame_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scion_types::srv::SendFrame_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const scion_types::srv::SendFrame_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scion_types::srv::SendFrame_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scion_types::srv::SendFrame_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scion_types::srv::SendFrame_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scion_types::srv::SendFrame_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scion_types::srv::SendFrame_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scion_types::srv::SendFrame_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scion_types::srv::SendFrame_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scion_types::srv::SendFrame_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scion_types__srv__SendFrame_Response
    std::shared_ptr<scion_types::srv::SendFrame_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scion_types__srv__SendFrame_Response
    std::shared_ptr<scion_types::srv::SendFrame_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendFrame_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendFrame_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendFrame_Response_

// alias to use template instance with default allocator
using SendFrame_Response =
  scion_types::srv::SendFrame_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace scion_types

namespace scion_types
{

namespace srv
{

struct SendFrame
{
  using Request = scion_types::srv::SendFrame_Request;
  using Response = scion_types::srv::SendFrame_Response;
};

}  // namespace srv

}  // namespace scion_types

#endif  // SCION_TYPES__SRV__DETAIL__SEND_FRAME__STRUCT_HPP_
