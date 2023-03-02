// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scion_types:msg/CanFrame.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__MSG__DETAIL__CAN_FRAME__STRUCT_H_
#define SCION_TYPES__MSG__DETAIL__CAN_FRAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/CanFrame in the package scion_types.
typedef struct scion_types__msg__CanFrame
{
  int32_t can_id;
  uint8_t can_data[8];
} scion_types__msg__CanFrame;

// Struct for a sequence of scion_types__msg__CanFrame.
typedef struct scion_types__msg__CanFrame__Sequence
{
  scion_types__msg__CanFrame * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scion_types__msg__CanFrame__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCION_TYPES__MSG__DETAIL__CAN_FRAME__STRUCT_H_
