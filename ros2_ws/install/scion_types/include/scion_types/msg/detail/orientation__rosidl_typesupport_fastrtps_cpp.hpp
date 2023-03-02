// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from scion_types:msg/Orientation.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__ORIENTATION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SCION_TYPES__MSG__DETAIL__ORIENTATION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "scion_types/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "scion_types/msg/detail/orientation__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace scion_types
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scion_types
cdr_serialize(
  const scion_types::msg::Orientation & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scion_types
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  scion_types::msg::Orientation & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scion_types
get_serialized_size(
  const scion_types::msg::Orientation & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scion_types
max_serialized_size_Orientation(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace scion_types

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_scion_types
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, scion_types, msg, Orientation)();

#ifdef __cplusplus
}
#endif

#endif  // SCION_TYPES__MSG__DETAIL__ORIENTATION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
