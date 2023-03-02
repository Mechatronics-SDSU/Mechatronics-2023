// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scion_types:msg/Orientation.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__ORIENTATION__STRUCT_H_
#define SCION_TYPES__MSG__DETAIL__ORIENTATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'orientation'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/Orientation in the package scion_types.
typedef struct scion_types__msg__Orientation
{
  rosidl_runtime_c__float__Sequence orientation;
} scion_types__msg__Orientation;

// Struct for a sequence of scion_types__msg__Orientation.
typedef struct scion_types__msg__Orientation__Sequence
{
  scion_types__msg__Orientation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scion_types__msg__Orientation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCION_TYPES__MSG__DETAIL__ORIENTATION__STRUCT_H_
