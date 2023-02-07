// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from can_msg_interfaces:msg/CanFrame.idl
// generated code does not contain a copyright notice

#ifndef CAN_MSG_INTERFACES__MSG__DETAIL__CAN_FRAME__STRUCT_H_
#define CAN_MSG_INTERFACES__MSG__DETAIL__CAN_FRAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/CanFrame in the package can_msg_interfaces.
typedef struct can_msg_interfaces__msg__CanFrame
{
  int32_t can_id;
  uint8_t can_data[8];
} can_msg_interfaces__msg__CanFrame;

// Struct for a sequence of can_msg_interfaces__msg__CanFrame.
typedef struct can_msg_interfaces__msg__CanFrame__Sequence
{
  can_msg_interfaces__msg__CanFrame * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} can_msg_interfaces__msg__CanFrame__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAN_MSG_INTERFACES__MSG__DETAIL__CAN_FRAME__STRUCT_H_
