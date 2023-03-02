// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from zed_interfaces:msg/Object.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/object__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "zed_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zed_interfaces/msg/detail/object__struct.h"
#include "zed_interfaces/msg/detail/object__functions.h"
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

#include "rosidl_runtime_c/string.h"  // label, sublabel
#include "rosidl_runtime_c/string_functions.h"  // label, sublabel
#include "zed_interfaces/msg/detail/bounding_box2_df__functions.h"  // head_bounding_box_2d
#include "zed_interfaces/msg/detail/bounding_box2_di__functions.h"  // bounding_box_2d
#include "zed_interfaces/msg/detail/bounding_box3_d__functions.h"  // bounding_box_3d, head_bounding_box_3d
#include "zed_interfaces/msg/detail/skeleton2_d__functions.h"  // skeleton_2d
#include "zed_interfaces/msg/detail/skeleton3_d__functions.h"  // skeleton_3d

// forward declare type support functions
size_t get_serialized_size_zed_interfaces__msg__BoundingBox2Df(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_zed_interfaces__msg__BoundingBox2Df(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox2Df)();
size_t get_serialized_size_zed_interfaces__msg__BoundingBox2Di(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_zed_interfaces__msg__BoundingBox2Di(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox2Di)();
size_t get_serialized_size_zed_interfaces__msg__BoundingBox3D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_zed_interfaces__msg__BoundingBox3D(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox3D)();
size_t get_serialized_size_zed_interfaces__msg__Skeleton2D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_zed_interfaces__msg__Skeleton2D(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Skeleton2D)();
size_t get_serialized_size_zed_interfaces__msg__Skeleton3D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_zed_interfaces__msg__Skeleton3D(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Skeleton3D)();


using _Object__ros_msg_type = zed_interfaces__msg__Object;

static bool _Object__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Object__ros_msg_type * ros_message = static_cast<const _Object__ros_msg_type *>(untyped_ros_message);
  // Field name: label
  {
    const rosidl_runtime_c__String * str = &ros_message->label;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: label_id
  {
    cdr << ros_message->label_id;
  }

  // Field name: sublabel
  {
    const rosidl_runtime_c__String * str = &ros_message->sublabel;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: confidence
  {
    cdr << ros_message->confidence;
  }

  // Field name: position
  {
    size_t size = 3;
    auto array_ptr = ros_message->position;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: position_covariance
  {
    size_t size = 6;
    auto array_ptr = ros_message->position_covariance;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: velocity
  {
    size_t size = 3;
    auto array_ptr = ros_message->velocity;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: tracking_available
  {
    cdr << (ros_message->tracking_available ? true : false);
  }

  // Field name: tracking_state
  {
    cdr << ros_message->tracking_state;
  }

  // Field name: action_state
  {
    cdr << ros_message->action_state;
  }

  // Field name: bounding_box_2d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox2Di
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->bounding_box_2d, cdr))
    {
      return false;
    }
  }

  // Field name: bounding_box_3d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox3D
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->bounding_box_3d, cdr))
    {
      return false;
    }
  }

  // Field name: dimensions_3d
  {
    size_t size = 3;
    auto array_ptr = ros_message->dimensions_3d;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: skeleton_available
  {
    cdr << (ros_message->skeleton_available ? true : false);
  }

  // Field name: body_format
  {
    cdr << ros_message->body_format;
  }

  // Field name: head_bounding_box_2d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox2Df
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->head_bounding_box_2d, cdr))
    {
      return false;
    }
  }

  // Field name: head_bounding_box_3d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox3D
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->head_bounding_box_3d, cdr))
    {
      return false;
    }
  }

  // Field name: head_position
  {
    size_t size = 3;
    auto array_ptr = ros_message->head_position;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: skeleton_2d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Skeleton2D
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->skeleton_2d, cdr))
    {
      return false;
    }
  }

  // Field name: skeleton_3d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Skeleton3D
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->skeleton_3d, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _Object__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Object__ros_msg_type * ros_message = static_cast<_Object__ros_msg_type *>(untyped_ros_message);
  // Field name: label
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->label.data) {
      rosidl_runtime_c__String__init(&ros_message->label);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->label,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'label'\n");
      return false;
    }
  }

  // Field name: label_id
  {
    cdr >> ros_message->label_id;
  }

  // Field name: sublabel
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->sublabel.data) {
      rosidl_runtime_c__String__init(&ros_message->sublabel);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->sublabel,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'sublabel'\n");
      return false;
    }
  }

  // Field name: confidence
  {
    cdr >> ros_message->confidence;
  }

  // Field name: position
  {
    size_t size = 3;
    auto array_ptr = ros_message->position;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: position_covariance
  {
    size_t size = 6;
    auto array_ptr = ros_message->position_covariance;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: velocity
  {
    size_t size = 3;
    auto array_ptr = ros_message->velocity;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: tracking_available
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->tracking_available = tmp ? true : false;
  }

  // Field name: tracking_state
  {
    cdr >> ros_message->tracking_state;
  }

  // Field name: action_state
  {
    cdr >> ros_message->action_state;
  }

  // Field name: bounding_box_2d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox2Di
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->bounding_box_2d))
    {
      return false;
    }
  }

  // Field name: bounding_box_3d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox3D
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->bounding_box_3d))
    {
      return false;
    }
  }

  // Field name: dimensions_3d
  {
    size_t size = 3;
    auto array_ptr = ros_message->dimensions_3d;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: skeleton_available
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->skeleton_available = tmp ? true : false;
  }

  // Field name: body_format
  {
    cdr >> ros_message->body_format;
  }

  // Field name: head_bounding_box_2d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox2Df
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->head_bounding_box_2d))
    {
      return false;
    }
  }

  // Field name: head_bounding_box_3d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, BoundingBox3D
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->head_bounding_box_3d))
    {
      return false;
    }
  }

  // Field name: head_position
  {
    size_t size = 3;
    auto array_ptr = ros_message->head_position;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: skeleton_2d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Skeleton2D
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->skeleton_2d))
    {
      return false;
    }
  }

  // Field name: skeleton_3d
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Skeleton3D
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->skeleton_3d))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t get_serialized_size_zed_interfaces__msg__Object(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Object__ros_msg_type * ros_message = static_cast<const _Object__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name label
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->label.size + 1);
  // field.name label_id
  {
    size_t item_size = sizeof(ros_message->label_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sublabel
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->sublabel.size + 1);
  // field.name confidence
  {
    size_t item_size = sizeof(ros_message->confidence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->position;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position_covariance
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->position_covariance;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velocity
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->velocity;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tracking_available
  {
    size_t item_size = sizeof(ros_message->tracking_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tracking_state
  {
    size_t item_size = sizeof(ros_message->tracking_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name action_state
  {
    size_t item_size = sizeof(ros_message->action_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bounding_box_2d

  current_alignment += get_serialized_size_zed_interfaces__msg__BoundingBox2Di(
    &(ros_message->bounding_box_2d), current_alignment);
  // field.name bounding_box_3d

  current_alignment += get_serialized_size_zed_interfaces__msg__BoundingBox3D(
    &(ros_message->bounding_box_3d), current_alignment);
  // field.name dimensions_3d
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->dimensions_3d;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name skeleton_available
  {
    size_t item_size = sizeof(ros_message->skeleton_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name body_format
  {
    size_t item_size = sizeof(ros_message->body_format);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name head_bounding_box_2d

  current_alignment += get_serialized_size_zed_interfaces__msg__BoundingBox2Df(
    &(ros_message->head_bounding_box_2d), current_alignment);
  // field.name head_bounding_box_3d

  current_alignment += get_serialized_size_zed_interfaces__msg__BoundingBox3D(
    &(ros_message->head_bounding_box_3d), current_alignment);
  // field.name head_position
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->head_position;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name skeleton_2d

  current_alignment += get_serialized_size_zed_interfaces__msg__Skeleton2D(
    &(ros_message->skeleton_2d), current_alignment);
  // field.name skeleton_3d

  current_alignment += get_serialized_size_zed_interfaces__msg__Skeleton3D(
    &(ros_message->skeleton_3d), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _Object__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_zed_interfaces__msg__Object(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t max_serialized_size_zed_interfaces__msg__Object(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: label
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: label_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: sublabel
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: confidence
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: position
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: position_covariance
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: velocity
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: tracking_available
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: tracking_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: action_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: bounding_box_2d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_zed_interfaces__msg__BoundingBox2Di(
        full_bounded, current_alignment);
    }
  }
  // member: bounding_box_3d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_zed_interfaces__msg__BoundingBox3D(
        full_bounded, current_alignment);
    }
  }
  // member: dimensions_3d
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: skeleton_available
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: body_format
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: head_bounding_box_2d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_zed_interfaces__msg__BoundingBox2Df(
        full_bounded, current_alignment);
    }
  }
  // member: head_bounding_box_3d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_zed_interfaces__msg__BoundingBox3D(
        full_bounded, current_alignment);
    }
  }
  // member: head_position
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: skeleton_2d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_zed_interfaces__msg__Skeleton2D(
        full_bounded, current_alignment);
    }
  }
  // member: skeleton_3d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_zed_interfaces__msg__Skeleton3D(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _Object__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_zed_interfaces__msg__Object(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Object = {
  "zed_interfaces::msg",
  "Object",
  _Object__cdr_serialize,
  _Object__cdr_deserialize,
  _Object__get_serialized_size,
  _Object__max_serialized_size
};

static rosidl_message_type_support_t _Object__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Object,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, msg, Object)() {
  return &_Object__type_support;
}

#if defined(__cplusplus)
}
#endif
