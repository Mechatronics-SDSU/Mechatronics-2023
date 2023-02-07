// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zed_interfaces:msg/PlaneStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__PLANE_STAMPED__STRUCT_HPP_
#define ZED_INTERFACES__MSG__DETAIL__PLANE_STAMPED__STRUCT_HPP_

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
// Member 'mesh'
#include "shape_msgs/msg/detail/mesh__struct.hpp"
// Member 'coefficients'
#include "shape_msgs/msg/detail/plane__struct.hpp"
// Member 'normal'
// Member 'center'
#include "geometry_msgs/msg/detail/point32__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/transform__struct.hpp"
// Member 'bounds'
#include "geometry_msgs/msg/detail/polygon__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__zed_interfaces__msg__PlaneStamped __attribute__((deprecated))
#else
# define DEPRECATED__zed_interfaces__msg__PlaneStamped __declspec(deprecated)
#endif

namespace zed_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PlaneStamped_
{
  using Type = PlaneStamped_<ContainerAllocator>;

  explicit PlaneStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    mesh(_init),
    coefficients(_init),
    normal(_init),
    center(_init),
    pose(_init),
    bounds(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 2>::iterator, float>(this->extents.begin(), this->extents.end(), 0.0f);
    }
  }

  explicit PlaneStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    mesh(_alloc, _init),
    coefficients(_alloc, _init),
    normal(_alloc, _init),
    center(_alloc, _init),
    pose(_alloc, _init),
    extents(_alloc),
    bounds(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 2>::iterator, float>(this->extents.begin(), this->extents.end(), 0.0f);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mesh_type =
    shape_msgs::msg::Mesh_<ContainerAllocator>;
  _mesh_type mesh;
  using _coefficients_type =
    shape_msgs::msg::Plane_<ContainerAllocator>;
  _coefficients_type coefficients;
  using _normal_type =
    geometry_msgs::msg::Point32_<ContainerAllocator>;
  _normal_type normal;
  using _center_type =
    geometry_msgs::msg::Point32_<ContainerAllocator>;
  _center_type center;
  using _pose_type =
    geometry_msgs::msg::Transform_<ContainerAllocator>;
  _pose_type pose;
  using _extents_type =
    std::array<float, 2>;
  _extents_type extents;
  using _bounds_type =
    geometry_msgs::msg::Polygon_<ContainerAllocator>;
  _bounds_type bounds;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mesh(
    const shape_msgs::msg::Mesh_<ContainerAllocator> & _arg)
  {
    this->mesh = _arg;
    return *this;
  }
  Type & set__coefficients(
    const shape_msgs::msg::Plane_<ContainerAllocator> & _arg)
  {
    this->coefficients = _arg;
    return *this;
  }
  Type & set__normal(
    const geometry_msgs::msg::Point32_<ContainerAllocator> & _arg)
  {
    this->normal = _arg;
    return *this;
  }
  Type & set__center(
    const geometry_msgs::msg::Point32_<ContainerAllocator> & _arg)
  {
    this->center = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Transform_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__extents(
    const std::array<float, 2> & _arg)
  {
    this->extents = _arg;
    return *this;
  }
  Type & set__bounds(
    const geometry_msgs::msg::Polygon_<ContainerAllocator> & _arg)
  {
    this->bounds = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zed_interfaces::msg::PlaneStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const zed_interfaces::msg::PlaneStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zed_interfaces::msg::PlaneStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zed_interfaces::msg::PlaneStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::PlaneStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::PlaneStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zed_interfaces::msg::PlaneStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zed_interfaces::msg::PlaneStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zed_interfaces::msg::PlaneStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zed_interfaces::msg::PlaneStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zed_interfaces__msg__PlaneStamped
    std::shared_ptr<zed_interfaces::msg::PlaneStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zed_interfaces__msg__PlaneStamped
    std::shared_ptr<zed_interfaces::msg::PlaneStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlaneStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mesh != other.mesh) {
      return false;
    }
    if (this->coefficients != other.coefficients) {
      return false;
    }
    if (this->normal != other.normal) {
      return false;
    }
    if (this->center != other.center) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->extents != other.extents) {
      return false;
    }
    if (this->bounds != other.bounds) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlaneStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlaneStamped_

// alias to use template instance with default allocator
using PlaneStamped =
  zed_interfaces::msg::PlaneStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zed_interfaces

#endif  // ZED_INTERFACES__MSG__DETAIL__PLANE_STAMPED__STRUCT_HPP_
