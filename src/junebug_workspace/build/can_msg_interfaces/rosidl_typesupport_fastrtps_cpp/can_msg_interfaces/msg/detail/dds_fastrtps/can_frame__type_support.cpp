// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from can_msg_interfaces:msg/CanFrame.idl
// generated code does not contain a copyright notice
#include "can_msg_interfaces/msg/detail/can_frame__rosidl_typesupport_fastrtps_cpp.hpp"
#include "can_msg_interfaces/msg/detail/can_frame__struct.hpp"

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

namespace can_msg_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_msg_interfaces
cdr_serialize(
  const can_msg_interfaces::msg::CanFrame & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: can_id
  cdr << ros_message.can_id;
  // Member: can_data
  {
    cdr << ros_message.can_data;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_msg_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  can_msg_interfaces::msg::CanFrame & ros_message)
{
  // Member: can_id
  cdr >> ros_message.can_id;

  // Member: can_data
  {
    cdr >> ros_message.can_data;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_msg_interfaces
get_serialized_size(
  const can_msg_interfaces::msg::CanFrame & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: can_id
  {
    size_t item_size = sizeof(ros_message.can_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: can_data
  {
    size_t array_size = 8;
    size_t item_size = sizeof(ros_message.can_data[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_can_msg_interfaces
max_serialized_size_CanFrame(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: can_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: can_data
  {
    size_t array_size = 8;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _CanFrame__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const can_msg_interfaces::msg::CanFrame *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _CanFrame__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<can_msg_interfaces::msg::CanFrame *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _CanFrame__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const can_msg_interfaces::msg::CanFrame *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _CanFrame__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_CanFrame(full_bounded, 0);
}

static message_type_support_callbacks_t _CanFrame__callbacks = {
  "can_msg_interfaces::msg",
  "CanFrame",
  _CanFrame__cdr_serialize,
  _CanFrame__cdr_deserialize,
  _CanFrame__get_serialized_size,
  _CanFrame__max_serialized_size
};

static rosidl_message_type_support_t _CanFrame__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_CanFrame__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace can_msg_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_can_msg_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<can_msg_interfaces::msg::CanFrame>()
{
  return &can_msg_interfaces::msg::typesupport_fastrtps_cpp::_CanFrame__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, can_msg_interfaces, msg, CanFrame)() {
  return &can_msg_interfaces::msg::typesupport_fastrtps_cpp::_CanFrame__handle;
}

#ifdef __cplusplus
}
#endif
