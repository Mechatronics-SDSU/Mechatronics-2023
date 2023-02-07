// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from zed_interfaces:msg/PlaneStamped.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/plane_stamped__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "zed_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zed_interfaces/msg/detail/plane_stamped__struct.h"
#include "zed_interfaces/msg/detail/plane_stamped__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/point32__functions.h"  // center, normal
#include "geometry_msgs/msg/detail/polygon__functions.h"  // bounds
#include "geometry_msgs/msg/detail/transform__functions.h"  // pose
#include "shape_msgs/msg/detail/mesh__functions.h"  // mesh
#include "shape_msgs/msg/detail/plane__functions.h"  // coefficients
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t get_serialized_size_geometry_msgs__msg__Point32(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t max_serialized_size_geometry_msgs__msg__Point32(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point32)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t get_serialized_size_geometry_msgs__msg__Polygon(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t max_serialized_size_geometry_msgs__msg__Polygon(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Polygon)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t get_serialized_size_geometry_msgs__msg__Transform(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t max_serialized_size_geometry_msgs__msg__Transform(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Transform)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t get_serialized_size_shape_msgs__msg__Mesh(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t max_serialized_size_shape_msgs__msg__Mesh(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, shape_msgs, msg, Mesh)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t get_serialized_size_shape_msgs__msg__Plane(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t max_serialized_size_shape_msgs__msg__Plane(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, shape_msgs, msg, Plane)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_zed_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _PlaneStamped__ros_msg_type = zed_interfaces__msg__PlaneStamped;

static bool _PlaneStamped__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PlaneStamped__ros_msg_type * ros_message = static_cast<const _PlaneStamped__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: mesh
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, shape_msgs, msg, Mesh
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->mesh, cdr))
    {
      return false;
    }
  }

  // Field name: coefficients
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, shape_msgs, msg, Plane
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->coefficients, cdr))
    {
      return false;
    }
  }

  // Field name: normal
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point32
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->normal, cdr))
    {
      return false;
    }
  }

  // Field name: center
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point32
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->center, cdr))
    {
      return false;
    }
  }

  // Field name: pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Transform
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pose, cdr))
    {
      return false;
    }
  }

  // Field name: extents
  {
    size_t size = 2;
    auto array_ptr = ros_message->extents;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: bounds
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Polygon
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->bounds, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _PlaneStamped__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PlaneStamped__ros_msg_type * ros_message = static_cast<_PlaneStamped__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: mesh
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, shape_msgs, msg, Mesh
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->mesh))
    {
      return false;
    }
  }

  // Field name: coefficients
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, shape_msgs, msg, Plane
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->coefficients))
    {
      return false;
    }
  }

  // Field name: normal
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point32
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->normal))
    {
      return false;
    }
  }

  // Field name: center
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point32
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->center))
    {
      return false;
    }
  }

  // Field name: pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Transform
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pose))
    {
      return false;
    }
  }

  // Field name: extents
  {
    size_t size = 2;
    auto array_ptr = ros_message->extents;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: bounds
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Polygon
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->bounds))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t get_serialized_size_zed_interfaces__msg__PlaneStamped(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlaneStamped__ros_msg_type * ros_message = static_cast<const _PlaneStamped__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name mesh

  current_alignment += get_serialized_size_shape_msgs__msg__Mesh(
    &(ros_message->mesh), current_alignment);
  // field.name coefficients

  current_alignment += get_serialized_size_shape_msgs__msg__Plane(
    &(ros_message->coefficients), current_alignment);
  // field.name normal

  current_alignment += get_serialized_size_geometry_msgs__msg__Point32(
    &(ros_message->normal), current_alignment);
  // field.name center

  current_alignment += get_serialized_size_geometry_msgs__msg__Point32(
    &(ros_message->center), current_alignment);
  // field.name pose

  current_alignment += get_serialized_size_geometry_msgs__msg__Transform(
    &(ros_message->pose), current_alignment);
  // field.name extents
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->extents;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bounds

  current_alignment += get_serialized_size_geometry_msgs__msg__Polygon(
    &(ros_message->bounds), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _PlaneStamped__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_zed_interfaces__msg__PlaneStamped(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t max_serialized_size_zed_interfaces__msg__PlaneStamped(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: mesh
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_shape_msgs__msg__Mesh(
        full_bounded, current_alignment);
    }
  }
  // member: coefficients
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_shape_msgs__msg__Plane(
        full_bounded, current_alignment);
    }
  }
  // member: normal
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Point32(
        full_bounded, current_alignment);
    }
  }
  // member: center
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Point32(
        full_bounded, current_alignment);
    }
  }
  // member: pose
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Transform(
        full_bounded, current_alignment);
    }
  }
  // member: extents
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: bounds
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Polygon(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _PlaneStamped__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_zed_interfaces__msg__PlaneStamped(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_PlaneStamped = {
  "zed_interfaces::msg",
  "PlaneStamped",
  _PlaneStamped__cdr_serialize,
  _PlaneStamped__cdr_deserialize,
  _PlaneStamped__get_serialized_size,
  _PlaneStamped__max_serialized_size
};

static rosidl_message_type_support_t _PlaneStamped__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PlaneStamped,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, PlaneStamped)() {
  return &_PlaneStamped__type_support;
}

#if defined(__cplusplus)
}
#endif
