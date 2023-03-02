// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:msg/Keypoint3D.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__MSG__DETAIL__KEYPOINT3_D__STRUCT_H_
#define ZED_INTERFACES__MSG__DETAIL__KEYPOINT3_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Keypoint3D in the package zed_interfaces.
typedef struct zed_interfaces__msg__Keypoint3D
{
  float kp[3];
} zed_interfaces__msg__Keypoint3D;

// Struct for a sequence of zed_interfaces__msg__Keypoint3D.
typedef struct zed_interfaces__msg__Keypoint3D__Sequence
{
  zed_interfaces__msg__Keypoint3D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__msg__Keypoint3D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__MSG__DETAIL__KEYPOINT3_D__STRUCT_H_
