// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from zed_interfaces:msg/Keypoint2Di.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "zed_interfaces/msg/detail/keypoint2_di__rosidl_typesupport_introspection_c.h"
#include "zed_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "zed_interfaces/msg/detail/keypoint2_di__functions.h"
#include "zed_interfaces/msg/detail/keypoint2_di__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zed_interfaces__msg__Keypoint2Di__init(message_memory);
}

void Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_fini_function(void * message_memory)
{
  zed_interfaces__msg__Keypoint2Di__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_message_member_array[1] = {
  {
    "kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces__msg__Keypoint2Di, kp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_message_members = {
  "zed_interfaces__msg",  // message namespace
  "Keypoint2Di",  // message name
  1,  // number of fields
  sizeof(zed_interfaces__msg__Keypoint2Di),
  Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_message_member_array,  // message members
  Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_init_function,  // function to initialize message memory (memory has to be allocated)
  Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_message_type_support_handle = {
  0,
  &Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zed_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, msg, Keypoint2Di)() {
  if (!Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_message_type_support_handle.typesupport_identifier) {
    Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Keypoint2Di__rosidl_typesupport_introspection_c__Keypoint2Di_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
