// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from zed_interfaces:msg/Skeleton3D.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/skeleton3_d__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "zed_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zed_interfaces/msg/detail/skeleton3_d__struct.h"
#include "zed_interfaces/msg/detail/skeleton3_d__functions.h"
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

#include "zed_interfaces/msg/detail/keypoint3_d__functions.h"  // keypoints

// forward declare type support functions
size_t get_serialized_size_zed_interfaces__msg__Keypoint3D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_zed_interfaces__msg__Keypoint3D(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Keypoint3D)();


using _Skeleton3D__ros_msg_type = zed_interfaces__msg__Skeleton3D;

static bool _Skeleton3D__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Skeleton3D__ros_msg_type * ros_message = static_cast<const _Skeleton3D__ros_msg_type *>(untyped_ros_message);
  // Field name: keypoints
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Keypoint3D
      )()->data);
    size_t size = 34;
    auto array_ptr = ros_message->keypoints;
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

static bool _Skeleton3D__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Skeleton3D__ros_msg_type * ros_message = static_cast<_Skeleton3D__ros_msg_type *>(untyped_ros_message);
  // Field name: keypoints
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Keypoint3D
      )()->data);
    size_t size = 34;
    auto array_ptr = ros_message->keypoints;
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
size_t get_serialized_size_zed_interfaces__msg__Skeleton3D(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Skeleton3D__ros_msg_type * ros_message = static_cast<const _Skeleton3D__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name keypoints
  {
    size_t array_size = 34;
    auto array_ptr = ros_message->keypoints;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_zed_interfaces__msg__Keypoint3D(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Skeleton3D__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_zed_interfaces__msg__Skeleton3D(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t max_serialized_size_zed_interfaces__msg__Skeleton3D(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: keypoints
  {
    size_t array_size = 34;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_zed_interfaces__msg__Keypoint3D(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _Skeleton3D__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_zed_interfaces__msg__Skeleton3D(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Skeleton3D = {
  "zed_interfaces::msg",
  "Skeleton3D",
  _Skeleton3D__cdr_serialize,
  _Skeleton3D__cdr_deserialize,
  _Skeleton3D__get_serialized_size,
  _Skeleton3D__max_serialized_size
};

static rosidl_message_type_support_t _Skeleton3D__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Skeleton3D,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Skeleton3D)() {
  return &_Skeleton3D__type_support;
}

#if defined(__cplusplus)
}
#endif
