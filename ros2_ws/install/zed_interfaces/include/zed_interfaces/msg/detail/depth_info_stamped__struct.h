// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:msg/DepthInfoStamped.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__STRUCT_H_
#define ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/DepthInfoStamped in the package zed_interfaces.
typedef struct zed_interfaces__msg__DepthInfoStamped
{
  std_msgs__msg__Header header;
  float min_depth;
  float max_depth;
} zed_interfaces__msg__DepthInfoStamped;

// Struct for a sequence of zed_interfaces__msg__DepthInfoStamped.
typedef struct zed_interfaces__msg__DepthInfoStamped__Sequence
{
  zed_interfaces__msg__DepthInfoStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__msg__DepthInfoStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__MSG__DETAIL__DEPTH_INFO_STAMPED__STRUCT_H_
