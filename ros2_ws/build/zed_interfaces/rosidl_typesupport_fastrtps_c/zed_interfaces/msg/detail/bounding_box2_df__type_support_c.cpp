// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from zed_interfaces:msg/BoundingBox2Df.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/bounding_box2_df__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "zed_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zed_interfaces/msg/detail/bounding_box2_df__struct.h"
#include "zed_interfaces/msg/detail/bounding_box2_df__functions.h"
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

#include "zed_interfaces/msg/detail/keypoint2_df__functions.h"  // corners

// forward declare type support functions
size_t get_serialized_size_zed_interfaces__msg__Keypoint2Df(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_zed_interfaces__msg__Keypoint2Df(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Keypoint2Df)();


using _BoundingBox2Df__ros_msg_type = zed_interfaces__msg__BoundingBox2Df;

static bool _BoundingBox2Df__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BoundingBox2Df__ros_msg_type * ros_message = static_cast<const _BoundingBox2Df__ros_msg_type *>(untyped_ros_message);
  // Field name: corners
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Keypoint2Df
      )()->data);
    size_t size = 4;
    auto array_ptr = ros_message->corners;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _BoundingBox2Df__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BoundingBox2Df__ros_msg_type * ros_message = static_cast<_BoundingBox2Df__ros_msg_type *>(untyped_ros_message);
  // Field name: corners
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Keypoint2Df
      )()->data);
    size_t size = 4;
    auto array_ptr = ros_message->corners;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t get_serialized_size_zed_interfaces__msg__BoundingBox2Df(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BoundingBox2Df__ros_msg_type * ros_message = static_cast<const _BoundingBox2Df__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name corners
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->corners;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_zed_interfaces__msg__Keypoint2Df(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BoundingBox2Df__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_zed_interfaces__msg__BoundingBox2Df(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t max_serialized_size_zed_interfaces__msg__BoundingBox2Df(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: corners
  {
    size_t array_size = 4;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_zed_interfaces__msg__Keypoint2Df(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _BoundingBox2Df__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_zed_interfaces__msg__BoundingBox2Df(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_BoundingBox2Df = {
  "zed_interfaces::msg",
  "BoundingBox2Df",
  _BoundingBox2Df__cdr_serialize,
  _BoundingBox2Df__cdr_deserialize,
  _BoundingBox2Df__get_serialized_size,
  _BoundingBox2Df__max_serialized_size
};

static rosidl_message_type_support_t _BoundingBox2Df__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BoundingBox2Df,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox2Df)() {
  return &_BoundingBox2Df__type_support;
}

#if defined(__cplusplus)
}
#endif
