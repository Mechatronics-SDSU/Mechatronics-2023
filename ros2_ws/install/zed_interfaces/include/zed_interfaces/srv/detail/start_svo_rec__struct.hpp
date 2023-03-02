// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:srv/StartSvoRec.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__SRV__DETAIL__START_SVO_REC__STRUCT_HPP_
#define ZED_INTERFACES__SRV__DETAIL__START_SVO_REC__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__zed_interfaces__srv__StartSvoRec_Request __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__srv__StartSvoRec_Request __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartSvoRec_Request_
{
  using Type = StartSvoRec_Request_<ContainerAllocator>;

  explicit StartSvoRec_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bitrate = 0ul;
      this->compression_mode = 0;
      this->target_framerate = 0ul;
      this->input_transcode = false;
      this->svo_filename = "";
    }
  }

  explicit StartSvoRec_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : svo_filename(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bitrate = 0ul;
      this->compression_mode = 0;
      this->target_framerate = 0ul;
      this->input_transcode = false;
      this->svo_filename = "";
    }
  }

  // field types and members
  using _bitrate_type =
    uint32_t;
  _bitrate_type bitrate;
  using _compression_mode_type =
    uint8_t;
  _compression_mode_type compression_mode;
  using _target_framerate_type =
    uint32_t;
  _target_framerate_type target_framerate;
  using _input_transcode_type =
    bool;
  _input_transcode_type input_transcode;
  using _svo_filename_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _svo_filename_type svo_filename;

  // setters for named parameter idiom
  Type & set__bitrate(
    const uint32_t & _arg)
  {
    this->bitrate = _arg;
    return *this;
  }
  Type & set__compression_mode(
    const uint8_t & _arg)
  {
    this->compression_mode = _arg;
    return *this;
  }
  Type & set__target_framerate(
    const uint32_t & _arg)
  {
    this->target_framerate = _arg;
    return *this;
  }
  Type & set__input_transcode(
    const bool & _arg)
  {
    this->input_transcode = _arg;
    return *this;
  }
  Type & set__svo_filename(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->svo_filename = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__srv__StartSvoRec_Request
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__srv__StartSvoRec_Request
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartSvoRec_Request_ & other) const
  {
    if (this->bitrate != other.bitrate) {
      return false;
    }
    if (this->compression_mode != other.compression_mode) {
      return false;
    }
    if (this->target_framerate != other.target_framerate) {
      return false;
    }
    if (this->input_transcode != other.input_transcode) {
      return false;
    }
    if (this->svo_filename != other.svo_filename) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartSvoRec_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartSvoRec_Request_

// alias to use template instance with default allocator
using StartSvoRec_Request =
  zed_interfaces::srv::StartSvoRec_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace zed_interfaces


#ifndef _WIN32
# define DEPRECATED__zed_interfaces__srv__StartSvoRec_Response __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__srv__StartSvoRec_Response __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartSvoRec_Response_
{
  using Type = StartSvoRec_Response_<ContainerAllocator>;

  explicit StartSvoRec_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit StartSvoRec_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__srv__StartSvoRec_Response
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__srv__StartSvoRec_Response
    std::shared_ptr<zed_interfaces::srv::StartSvoRec_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartSvoRec_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartSvoRec_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartSvoRec_Response_

// alias to use template instance with default allocator
using StartSvoRec_Response =
  zed_interfaces::srv::StartSvoRec_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace zed_interfaces

namespace zed_interfaces
{

namespace srv
{

struct StartSvoRec
{
  using Request = zed_interfaces::srv::StartSvoRec_Request;
  using Response = zed_interfaces::srv::StartSvoRec_Response;
};

}  // namespace srv

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__SRV__DETAIL__START_SVO_REC__STRUCT_HPP_
