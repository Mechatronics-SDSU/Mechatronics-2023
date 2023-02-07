// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from zed_interfaces:msg/Object.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/object__rosidl_typesupport_fastrtps_cpp.hpp"
#include "zed_interfaces/msg/detail/object__struct.hpp"

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
namespace zed_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const zed_interfaces::msg::BoundingBox2Di &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  zed_interfaces::msg::BoundingBox2Di &);
size_t get_serialized_size(
  const zed_interfaces::msg::BoundingBox2Di &,
  size_t current_alignment);
size_t
max_serialized_size_BoundingBox2Di(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace zed_interfaces

namespace zed_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const zed_interfaces::msg::BoundingBox3D &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  zed_interfaces::msg::BoundingBox3D &);
size_t get_serialized_size(
  const zed_interfaces::msg::BoundingBox3D &,
  size_t current_alignment);
size_t
max_serialized_size_BoundingBox3D(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace zed_interfaces

namespace zed_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const zed_interfaces::msg::BoundingBox2Df &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  zed_interfaces::msg::BoundingBox2Df &);
size_t get_serialized_size(
  const zed_interfaces::msg::BoundingBox2Df &,
  size_t current_alignment);
size_t
max_serialized_size_BoundingBox2Df(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace zed_interfaces

namespace zed_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const zed_interfaces::msg::BoundingBox3D &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  zed_interfaces::msg::BoundingBox3D &);
size_t get_serialized_size(
  const zed_interfaces::msg::BoundingBox3D &,
  size_t current_alignment);
size_t
max_serialized_size_BoundingBox3D(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace zed_interfaces

namespace zed_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const zed_interfaces::msg::Skeleton2D &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  zed_interfaces::msg::Skeleton2D &);
size_t get_serialized_size(
  const zed_interfaces::msg::Skeleton2D &,
  size_t current_alignment);
size_t
max_serialized_size_Skeleton2D(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace zed_interfaces

namespace zed_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const zed_interfaces::msg::Skeleton3D &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  zed_interfaces::msg::Skeleton3D &);
size_t get_serialized_size(
  const zed_interfaces::msg::Skeleton3D &,
  size_t current_alignment);
size_t
max_serialized_size_Skeleton3D(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace zed_interfaces


namespace zed_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
cdr_serialize(
  const zed_interfaces::msg::Object & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: label
  cdr << ros_message.label;
  // Member: label_id
  cdr << ros_message.label_id;
  // Member: sublabel
  cdr << ros_message.sublabel;
  // Member: confidence
  cdr << ros_message.confidence;
  // Member: position
  {
    cdr << ros_message.position;
  }
  // Member: position_covariance
  {
    cdr << ros_message.position_covariance;
  }
  // Member: velocity
  {
    cdr << ros_message.velocity;
  }
  // Member: tracking_available
  cdr << (ros_message.tracking_available ? true : false);
  // Member: tracking_state
  cdr << ros_message.tracking_state;
  // Member: action_state
  cdr << ros_message.action_state;
  // Member: bounding_box_2d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.bounding_box_2d,
    cdr);
  // Member: bounding_box_3d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.bounding_box_3d,
    cdr);
  // Member: dimensions_3d
  {
    cdr << ros_message.dimensions_3d;
  }
  // Member: skeleton_available
  cdr << (ros_message.skeleton_available ? true : false);
  // Member: body_format
  cdr << ros_message.body_format;
  // Member: head_bounding_box_2d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.head_bounding_box_2d,
    cdr);
  // Member: head_bounding_box_3d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.head_bounding_box_3d,
    cdr);
  // Member: head_position
  {
    cdr << ros_message.head_position;
  }
  // Member: skeleton_2d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.skeleton_2d,
    cdr);
  // Member: skeleton_3d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.skeleton_3d,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  zed_interfaces::msg::Object & ros_message)
{
  // Member: label
  cdr >> ros_message.label;

  // Member: label_id
  cdr >> ros_message.label_id;

  // Member: sublabel
  cdr >> ros_message.sublabel;

  // Member: confidence
  cdr >> ros_message.confidence;

  // Member: position
  {
    cdr >> ros_message.position;
  }

  // Member: position_covariance
  {
    cdr >> ros_message.position_covariance;
  }

  // Member: velocity
  {
    cdr >> ros_message.velocity;
  }

  // Member: tracking_available
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.tracking_available = tmp ? true : false;
  }

  // Member: tracking_state
  cdr >> ros_message.tracking_state;

  // Member: action_state
  cdr >> ros_message.action_state;

  // Member: bounding_box_2d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.bounding_box_2d);

  // Member: bounding_box_3d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.bounding_box_3d);

  // Member: dimensions_3d
  {
    cdr >> ros_message.dimensions_3d;
  }

  // Member: skeleton_available
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.skeleton_available = tmp ? true : false;
  }

  // Member: body_format
  cdr >> ros_message.body_format;

  // Member: head_bounding_box_2d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.head_bounding_box_2d);

  // Member: head_bounding_box_3d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.head_bounding_box_3d);

  // Member: head_position
  {
    cdr >> ros_message.head_position;
  }

  // Member: skeleton_2d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.skeleton_2d);

  // Member: skeleton_3d
  zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.skeleton_3d);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
get_serialized_size(
  const zed_interfaces::msg::Object & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: label
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.label.size() + 1);
  // Member: label_id
  {
    size_t item_size = sizeof(ros_message.label_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sublabel
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.sublabel.size() + 1);
  // Member: confidence
  {
    size_t item_size = sizeof(ros_message.confidence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position_covariance
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.position_covariance[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: velocity
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.velocity[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tracking_available
  {
    size_t item_size = sizeof(ros_message.tracking_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tracking_state
  {
    size_t item_size = sizeof(ros_message.tracking_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: action_state
  {
    size_t item_size = sizeof(ros_message.action_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bounding_box_2d

  current_alignment +=
    zed_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.bounding_box_2d, current_alignment);
  // Member: bounding_box_3d

  current_alignment +=
    zed_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.bounding_box_3d, current_alignment);
  // Member: dimensions_3d
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.dimensions_3d[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: skeleton_available
  {
    size_t item_size = sizeof(ros_message.skeleton_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: body_format
  {
    size_t item_size = sizeof(ros_message.body_format);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: head_bounding_box_2d

  current_alignment +=
    zed_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.head_bounding_box_2d, current_alignment);
  // Member: head_bounding_box_3d

  current_alignment +=
    zed_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.head_bounding_box_3d, current_alignment);
  // Member: head_position
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.head_position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: skeleton_2d

  current_alignment +=
    zed_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.skeleton_2d, current_alignment);
  // Member: skeleton_3d

  current_alignment +=
    zed_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.skeleton_3d, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
max_serialized_size_Object(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: label
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: label_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: sublabel
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: confidence
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: position
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: position_covariance
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: velocity
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: tracking_available
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: tracking_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: action_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: bounding_box_2d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        zed_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_BoundingBox2Di(
        full_bounded, current_alignment);
    }
  }

  // Member: bounding_box_3d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        zed_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_BoundingBox3D(
        full_bounded, current_alignment);
    }
  }

  // Member: dimensions_3d
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: skeleton_available
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: body_format
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: head_bounding_box_2d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        zed_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_BoundingBox2Df(
        full_bounded, current_alignment);
    }
  }

  // Member: head_bounding_box_3d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        zed_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_BoundingBox3D(
        full_bounded, current_alignment);
    }
  }

  // Member: head_position
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: skeleton_2d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        zed_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Skeleton2D(
        full_bounded, current_alignment);
    }
  }

  // Member: skeleton_3d
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        zed_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Skeleton3D(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _Object__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const zed_interfaces::msg::Object *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Object__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<zed_interfaces::msg::Object *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Object__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const zed_interfaces::msg::Object *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Object__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Object(full_bounded, 0);
}

static message_type_support_callbacks_t _Object__callbacks = {
  "zed_interfaces::msg",
  "Object",
  _Object__cdr_serialize,
  _Object__cdr_deserialize,
  _Object__get_serialized_size,
  _Object__max_serialized_size
};

static rosidl_message_type_support_t _Object__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Object__callbacks,
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
get_message_type_support_handle<zed_interfaces::msg::Object>()
{
  return &zed_interfaces::msg::typesupport_fastrtps_cpp::_Object__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, zed_interfaces, msg, Object)() {
  return &zed_interfaces::msg::typesupport_fastrtps_cpp::_Object__handle;
}

#ifdef __cplusplus
}
#endif
