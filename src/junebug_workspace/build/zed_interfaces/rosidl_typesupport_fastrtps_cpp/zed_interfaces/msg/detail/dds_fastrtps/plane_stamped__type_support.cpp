// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from zed_interfaces:msg/PlaneStamped.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/plane_stamped__rosidl_typesupport_fastrtps_cpp.hpp"
#include "zed_interfaces/msg/detail/plane_stamped__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace shape_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const shape_msgs::msg::Mesh &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  shape_msgs::msg::Mesh &);
size_t get_serialized_size(
  const shape_msgs::msg::Mesh &,
  size_t current_alignment);
size_t
max_serialized_size_Mesh(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace shape_msgs

namespace shape_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const shape_msgs::msg::Plane &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  shape_msgs::msg::Plane &);
size_t get_serialized_size(
  const shape_msgs::msg::Plane &,
  size_t current_alignment);
size_t
max_serialized_size_Plane(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace shape_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Point32 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Point32 &);
size_t get_serialized_size(
  const geometry_msgs::msg::Point32 &,
  size_t current_alignment);
size_t
max_serialized_size_Point32(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Point32 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Point32 &);
size_t get_serialized_size(
  const geometry_msgs::msg::Point32 &,
  size_t current_alignment);
size_t
max_serialized_size_Point32(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Transform &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Transform &);
size_t get_serialized_size(
  const geometry_msgs::msg::Transform &,
  size_t current_alignment);
size_t
max_serialized_size_Transform(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Polygon &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Polygon &);
size_t get_serialized_size(
  const geometry_msgs::msg::Polygon &,
  size_t current_alignment);
size_t
max_serialized_size_Polygon(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs


namespace zed_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
cdr_serialize(
  const zed_interfaces::msg::PlaneStamped & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: mesh
  shape_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.mesh,
    cdr);
  // Member: coefficients
  shape_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.coefficients,
    cdr);
  // Member: normal
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.normal,
    cdr);
  // Member: center
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.center,
    cdr);
  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pose,
    cdr);
  // Member: extents
  {
    cdr << ros_message.extents;
  }
  // Member: bounds
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.bounds,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  zed_interfaces::msg::PlaneStamped & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: mesh
  shape_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.mesh);

  // Member: coefficients
  shape_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.coefficients);

  // Member: normal
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.normal);

  // Member: center
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.center);

  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pose);

  // Member: extents
  {
    cdr >> ros_message.extents;
  }

  // Member: bounds
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.bounds);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
get_serialized_size(
  const zed_interfaces::msg::PlaneStamped & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: mesh

  current_alignment +=
    shape_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.mesh, current_alignment);
  // Member: coefficients

  current_alignment +=
    shape_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.coefficients, current_alignment);
  // Member: normal

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.normal, current_alignment);
  // Member: center

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.center, current_alignment);
  // Member: pose

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pose, current_alignment);
  // Member: extents
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.extents[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bounds

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.bounds, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
max_serialized_size_PlaneStamped(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: mesh
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        shape_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Mesh(
        full_bounded, current_alignment);
    }
  }

  // Member: coefficients
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        shape_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Plane(
        full_bounded, current_alignment);
    }
  }

  // Member: normal
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point32(
        full_bounded, current_alignment);
    }
  }

  // Member: center
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point32(
        full_bounded, current_alignment);
    }
  }

  // Member: pose
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Transform(
        full_bounded, current_alignment);
    }
  }

  // Member: extents
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: bounds
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Polygon(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _PlaneStamped__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const zed_interfaces::msg::PlaneStamped *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PlaneStamped__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<zed_interfaces::msg::PlaneStamped *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PlaneStamped__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const zed_interfaces::msg::PlaneStamped *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PlaneStamped__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_PlaneStamped(full_bounded, 0);
}

static message_type_support_callbacks_t _PlaneStamped__callbacks = {
  "zed_interfaces::msg",
  "PlaneStamped",
  _PlaneStamped__cdr_serialize,
  _PlaneStamped__cdr_deserialize,
  _PlaneStamped__get_serialized_size,
  _PlaneStamped__max_serialized_size
};

static rosidl_message_type_support_t _PlaneStamped__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PlaneStamped__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace zed_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_zed_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<zed_interfaces::msg::PlaneStamped>()
{
  return &zed_interfaces::msg::typesupport_fastrtps_cpp::_PlaneStamped__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, zed_interfaces, msg, PlaneStamped)() {
  return &zed_interfaces::msg::typesupport_fastrtps_cpp::_PlaneStamped__handle;
}

#ifdef __cplusplus
}
#endif
