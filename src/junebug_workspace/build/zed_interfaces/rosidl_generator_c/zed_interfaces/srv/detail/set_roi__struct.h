// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_interfaces:srv/SetROI.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__SRV__DETAIL__SET_ROI__STRUCT_H_
#define ZED_INTERFACES__SRV__DETAIL__SET_ROI__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'roi'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetROI in the package zed_interfaces.
typedef struct zed_interfaces__srv__SetROI_Request
{
  rosidl_runtime_c__String roi;
} zed_interfaces__srv__SetROI_Request;

// Struct for a sequence of zed_interfaces__srv__SetROI_Request.
typedef struct zed_interfaces__srv__SetROI_Request__Sequence
{
  zed_interfaces__srv__SetROI_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__srv__SetROI_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

// Struct defined in srv/SetROI in the package zed_interfaces.
typedef struct zed_interfaces__srv__SetROI_Response
{
  bool success;
  rosidl_runtime_c__String message;
} zed_interfaces__srv__SetROI_Response;

// Struct for a sequence of zed_interfaces__srv__SetROI_Response.
typedef struct zed_interfaces__srv__SetROI_Response__Sequence
{
  zed_interfaces__srv__SetROI_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_interfaces__srv__SetROI_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_INTERFACES__SRV__DETAIL__SET_ROI__STRUCT_H_
