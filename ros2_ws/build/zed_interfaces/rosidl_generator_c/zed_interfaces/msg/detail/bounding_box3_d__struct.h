// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:msg/BoundingBox3D.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX3_D__STRUCT_H_
#define ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX3_D__STRUCT_H_

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
#include "zed_interfaces/msg/detail/keypoint3_d__struct.h"

// Struct defined in msg/BoundingBox3D in the package zed_interfaces.
typedef struct zed_interfaces__msg__BoundingBox3D
{
  zed_interfaces__msg__Keypoint3D corners[8];
} zed_interfaces__msg__BoundingBox3D;

// Struct for a sequence of zed_interfaces__msg__BoundingBox3D.
typedef struct zed_interfaces__msg__BoundingBox3D__Sequence
{
  zed_interfaces__msg__BoundingBox3D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__msg__BoundingBox3D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__MSG__DETAIL__BOUNDING_BOX3_D__STRUCT_H_
