// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:msg/BoundingBox2Di.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DI__STRUCT_H_
#define ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DI__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'corners'
#include "zed_interfaces/msg/detail/keypoint2_di__struct.h"

// Struct defined in msg/BoundingBox2Di in the package zed_interfaces.
typedef struct zed_interfaces__msg__BoundingBox2Di
{
  zed_interfaces__msg__Keypoint2Di corners[4];
} zed_interfaces__msg__BoundingBox2Di;

// Struct for a sequence of zed_interfaces__msg__BoundingBox2Di.
typedef struct zed_interfaces__msg__BoundingBox2Di__Sequence
{
  zed_interfaces__msg__BoundingBox2Di * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__msg__BoundingBox2Di__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX2_DI__STRUCT_H_
