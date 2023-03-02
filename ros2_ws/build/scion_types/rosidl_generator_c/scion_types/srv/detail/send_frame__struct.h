// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scion_types:srv/SendFrame.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__SRV__DETAIL__SEND_FRAME__STRUCT_H_
#define SCION_TYPES__SRV__DETAIL__SEND_FRAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/SendFrame in the package scion_types.
typedef struct scion_types__srv__SendFrame_Request
{
  int32_t can_id;
  int8_t can_dlc;
  uint8_t can_data[8];
} scion_types__srv__SendFrame_Request;

// Struct for a sequence of scion_types__srv__SendFrame_Request.
typedef struct scion_types__srv__SendFrame_Request__Sequence
{
  scion_types__srv__SendFrame_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scion_types__srv__SendFrame_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SendFrame in the package scion_types.
typedef struct scion_types__srv__SendFrame_Response
{
  int8_t status;
} scion_types__srv__SendFrame_Response;

// Struct for a sequence of scion_types__srv__SendFrame_Response.
typedef struct scion_types__srv__SendFrame_Response__Sequence
{
  scion_types__srv__SendFrame_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scion_types__srv__SendFrame_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCION_TYPES__SRV__DETAIL__SEND_FRAME__STRUCT_H_
