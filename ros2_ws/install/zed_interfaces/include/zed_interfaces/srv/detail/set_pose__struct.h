// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:srv/SetPose.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__SRV__DETAIL__SET_POSE__STRUCT_H_
#define ZED_INTERFACES__SRV__DETAIL__SET_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/SetPose in the package zed_interfaces.
typedef struct zed_interfaces__srv__SetPose_Request
{
  float pos[3];
  float orient[3];
} zed_interfaces__srv__SetPose_Request;

// Struct for a sequence of zed_interfaces__srv__SetPose_Request.
typedef struct zed_interfaces__srv__SetPose_Request__Sequence
{
  zed_interfaces__srv__SetPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__srv__SetPose_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetPose in the package zed_interfaces.
typedef struct zed_interfaces__srv__SetPose_Response
{
  bool success;
  rosidl_runtime_c__String message;
} zed_interfaces__srv__SetPose_Response;

// Struct for a sequence of zed_interfaces__srv__SetPose_Response.
typedef struct zed_interfaces__srv__SetPose_Response__Sequence
{
  zed_interfaces__srv__SetPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__srv__SetPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__SRV__DETAIL__SET_POSE__STRUCT_H_
