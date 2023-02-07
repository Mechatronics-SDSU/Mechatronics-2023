// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from zed_interfaces:msg/Skeleton2D.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/skeleton2_d__rosidl_typesupport_fastrtps_cpp.hpp"
#include "zed_interfaces/msg/detail/skeleton2_d__struct.hpp"

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
  const zed_interfaces::msg::Keypoint2Df &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  zed_interfaces::msg::Keypoint2Df &);
size_t get_serialized_size(
  const zed_interfaces::msg::Keypoint2Df &,
  size_t current_alignment);
size_t
max_serialized_size_Keypoint2Df(
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
  const zed_interfaces::msg::Skeleton2D & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: keypoints
  {
    for (size_t i = 0; i < 34; i++) {
      zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.keypoints[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  zed_interfaces::msg::Skeleton2D & ros_message)
{
  // Member: keypoints
  {
    for (size_t i = 0; i < 34; i++) {
      zed_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.keypoints[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
get_serialized_size(
  const zed_interfaces::msg::Skeleton2D & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: keypoints
  {
    size_t array_size = 34;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        zed_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.keypoints[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zed_interfaces
max_serialized_size_Skeleton2D(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: keypoints
  {
    size_t array_size = 34;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        zed_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Keypoint2Df(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _Skeleton2D__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const zed_interfaces::msg::Skeleton2D *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Skeleton2D__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<zed_interfaces::msg::Skeleton2D *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Skeleton2D__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const zed_interfaces::msg::Skeleton2D *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Skeleton2D__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Skeleton2D(full_bounded, 0);
}

static message_type_support_callbacks_t _Skeleton2D__callbacks = {
  "zed_interfaces::msg",
  "Skeleton2D",
  _Skeleton2D__cdr_serialize,
  _Skeleton2D__cdr_deserialize,
  _Skeleton2D__get_serialized_size,
  _Skeleton2D__max_serialized_size
};

static rosidl_message_type_support_t _Skeleton2D__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Skeleton2D__callbacks,
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
get_message_type_support_handle<zed_interfaces::msg::Skeleton2D>()
{
  return &zed_interfaces::msg::typesupport_fastrtps_cpp::_Skeleton2D__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, zed_interfaces, msg, Skeleton2D)() {
  return &zed_interfaces::msg::typesupport_fastrtps_cpp::_Skeleton2D__handle;
}

#ifdef __cplusplus
}
#endif
