// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scion_types:msg/DesiredState.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__DESIRED_STATE__STRUCT_H_
#define SCION_TYPES__MSG__DETAIL__DESIRED_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'desired_state'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/DesiredState in the package scion_types.
typedef struct scion_types__msg__DesiredState
{
  rosidl_runtime_c__float__Sequence desired_state;
} scion_types__msg__DesiredState;

// Struct for a sequence of scion_types__msg__DesiredState.
typedef struct scion_types__msg__DesiredState__Sequence
{
  scion_types__msg__DesiredState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scion_types__msg__DesiredState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCION_TYPES__MSG__DETAIL__DESIRED_STATE__STRUCT_H_
