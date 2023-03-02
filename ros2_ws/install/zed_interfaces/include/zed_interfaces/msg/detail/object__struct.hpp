// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_HPP_
#define ZED_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'bounding_box_2d'
#include "zed_interfaces/msg/detail/bounding_box2_di__struct.hpp"
// Member 'bounding_box_3d'
// Member 'head_bounding_box_3d'
#include "zed_interfaces/msg/detail/bounding_box3_d__struct.hpp"
// Member 'head_bounding_box_2d'
#include "zed_interfaces/msg/detail/bounding_box2_df__struct.hpp"
// Member 'skeleton_2d'
#include "zed_interfaces/msg/detail/skeleton2_d__struct.hpp"
// Member 'skeleton_3d'
#include "zed_interfaces/msg/detail/skeleton3_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__zed_interfaces__msg__Object __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__msg__Object __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Object_
{
  using Type = Object_<ContainerAllocator>;

  explicit Object_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : bounding_box_2d(_init),
    bounding_box_3d(_init),
    head_bounding_box_2d(_init),
    head_bounding_box_3d(_init),
    skeleton_2d(_init),
    skeleton_3d(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->label_id = 0;
      this->sublabel = "";
      this->confidence = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->position_covariance.begin(), this->position_covariance.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->velocity.begin(), this->velocity.end(), 0.0f);
      this->tracking_available = false;
      this->tracking_state = 0;
      this->action_state = 0;
      std::fill<typename std::array<float, 3>::iterator, float>(this->dimensions_3d.begin(), this->dimensions_3d.end(), 0.0f);
      this->skeleton_available = false;
      this->body_format = 0;
      std::fill<typename std::array<float, 3>::iterator, float>(this->head_position.begin(), this->head_position.end(), 0.0f);
    }
  }

  explicit Object_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : label(_alloc),
    sublabel(_alloc),
    position(_alloc),
    position_covariance(_alloc),
    velocity(_alloc),
    bounding_box_2d(_alloc, _init),
    bounding_box_3d(_alloc, _init),
    dimensions_3d(_alloc),
    head_bounding_box_2d(_alloc, _init),
    head_bounding_box_3d(_alloc, _init),
    head_position(_alloc),
    skeleton_2d(_alloc, _init),
    skeleton_3d(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->label_id = 0;
      this->sublabel = "";
      this->confidence = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->position_covariance.begin(), this->position_covariance.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->velocity.begin(), this->velocity.end(), 0.0f);
      this->tracking_available = false;
      this->tracking_state = 0;
      this->action_state = 0;
      std::fill<typename std::array<float, 3>::iterator, float>(this->dimensions_3d.begin(), this->dimensions_3d.end(), 0.0f);
      this->skeleton_available = false;
      this->body_format = 0;
      std::fill<typename std::array<float, 3>::iterator, float>(this->head_position.begin(), this->head_position.end(), 0.0f);
    }
  }

  // field types and members
  using _label_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _label_type label;
  using _label_id_type =
    int16_t;
  _label_id_type label_id;
  using _sublabel_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _sublabel_type sublabel;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _position_type =
    std::array<float, 3>;
  _position_type position;
  using _position_covariance_type =
    std::array<float, 6>;
  _position_covariance_type position_covariance;
  using _velocity_type =
    std::array<float, 3>;
  _velocity_type velocity;
  using _tracking_available_type =
    bool;
  _tracking_available_type tracking_available;
  using _tracking_state_type =
    int8_t;
  _tracking_state_type tracking_state;
  using _action_state_type =
    int8_t;
  _action_state_type action_state;
  using _bounding_box_2d_type =
    zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator>;
  _bounding_box_2d_type bounding_box_2d;
  using _bounding_box_3d_type =
    zed_interfaces::msg::BoundingBox3D_<ContainerAllocator>;
  _bounding_box_3d_type bounding_box_3d;
  using _dimensions_3d_type =
    std::array<float, 3>;
  _dimensions_3d_type dimensions_3d;
  using _skeleton_available_type =
    bool;
  _skeleton_available_type skeleton_available;
  using _body_format_type =
    int8_t;
  _body_format_type body_format;
  using _head_bounding_box_2d_type =
    zed_interfaces::msg::BoundingBox2Df_<ContainerAllocator>;
  _head_bounding_box_2d_type head_bounding_box_2d;
  using _head_bounding_box_3d_type =
    zed_interfaces::msg::BoundingBox3D_<ContainerAllocator>;
  _head_bounding_box_3d_type head_bounding_box_3d;
  using _head_position_type =
    std::array<float, 3>;
  _head_position_type head_position;
  using _skeleton_2d_type =
    zed_interfaces::msg::Skeleton2D_<ContainerAllocator>;
  _skeleton_2d_type skeleton_2d;
  using _skeleton_3d_type =
    zed_interfaces::msg::Skeleton3D_<ContainerAllocator>;
  _skeleton_3d_type skeleton_3d;

  // setters for named parameter idiom
  Type & set__label(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__label_id(
    const int16_t & _arg)
  {
    this->label_id = _arg;
    return *this;
  }
  Type & set__sublabel(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->sublabel = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__position(
    const std::array<float, 3> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__position_covariance(
    const std::array<float, 6> & _arg)
  {
    this->position_covariance = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::array<float, 3> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__tracking_available(
    const bool & _arg)
  {
    this->tracking_available = _arg;
    return *this;
  }
  Type & set__tracking_state(
    const int8_t & _arg)
  {
    this->tracking_state = _arg;
    return *this;
  }
  Type & set__action_state(
    const int8_t & _arg)
  {
    this->action_state = _arg;
    return *this;
  }
  Type & set__bounding_box_2d(
    const zed_interfaces::msg::BoundingBox2Di_<ContainerAllocator> & _arg)
  {
    this->bounding_box_2d = _arg;
    return *this;
  }
  Type & set__bounding_box_3d(
    const zed_interfaces::msg::BoundingBox3D_<ContainerAllocator> & _arg)
  {
    this->bounding_box_3d = _arg;
    return *this;
  }
  Type & set__dimensions_3d(
    const std::array<float, 3> & _arg)
  {
    this->dimensions_3d = _arg;
    return *this;
  }
  Type & set__skeleton_available(
    const bool & _arg)
  {
    this->skeleton_available = _arg;
    return *this;
  }
  Type & set__body_format(
    const int8_t & _arg)
  {
    this->body_format = _arg;
    return *this;
  }
  Type & set__head_bounding_box_2d(
    const zed_interfaces::msg::BoundingBox2Df_<ContainerAllocator> & _arg)
  {
    this->head_bounding_box_2d = _arg;
    return *this;
  }
  Type & set__head_bounding_box_3d(
    const zed_interfaces::msg::BoundingBox3D_<ContainerAllocator> & _arg)
  {
    this->head_bounding_box_3d = _arg;
    return *this;
  }
  Type & set__head_position(
    const std::array<float, 3> & _arg)
  {
    this->head_position = _arg;
    return *this;
  }
  Type & set__skeleton_2d(
    const zed_interfaces::msg::Skeleton2D_<ContainerAllocator> & _arg)
  {
    this->skeleton_2d = _arg;
    return *this;
  }
  Type & set__skeleton_3d(
    const zed_interfaces::msg::Skeleton3D_<ContainerAllocator> & _arg)
  {
    this->skeleton_3d = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::msg::Object_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::msg::Object_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::msg::Object_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::msg::Object_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::Object_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::Object_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::Object_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::Object_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::msg::Object_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::msg::Object_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__msg__Object
    std::shared_ptr<zed_interfaces::msg::Object_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__msg__Object
    std::shared_ptr<zed_interfaces::msg::Object_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Object_ & other) const
  {
    if (this->label != other.label) {
      return false;
    }
    if (this->label_id != other.label_id) {
      return false;
    }
    if (this->sublabel != other.sublabel) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->position_covariance != other.position_covariance) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->tracking_available != other.tracking_available) {
      return false;
    }
    if (this->tracking_state != other.tracking_state) {
      return false;
    }
    if (this->action_state != other.action_state) {
      return false;
    }
    if (this->bounding_box_2d != other.bounding_box_2d) {
      return false;
    }
    if (this->bounding_box_3d != other.bounding_box_3d) {
      return false;
    }
    if (this->dimensions_3d != other.dimensions_3d) {
      return false;
    }
    if (this->skeleton_available != other.skeleton_available) {
      return false;
    }
    if (this->body_format != other.body_format) {
      return false;
    }
    if (this->head_bounding_box_2d != other.head_bounding_box_2d) {
      return false;
    }
    if (this->head_bounding_box_3d != other.head_bounding_box_3d) {
      return false;
    }
    if (this->head_position != other.head_position) {
      return false;
    }
    if (this->skeleton_2d != other.skeleton_2d) {
      return false;
    }
    if (this->skeleton_3d != other.skeleton_3d) {
      return false;
    }
    return true;
  }
  bool operator!=(const Object_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Object_

// alias to use template instance with default allocator
using Object =
  zed_interfaces::msg::Object_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__OBJECT__STRUCT_HPP_
