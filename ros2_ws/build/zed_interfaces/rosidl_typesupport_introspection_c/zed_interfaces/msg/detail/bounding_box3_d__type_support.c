// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from zed_interfaces:msg/BoundingBox3D.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "zed_interfaces/msg/detail/bounding_box3_d__rosidl_typesupport_introspection_c.h"
#include "zed_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "zed_interfaces/msg/detail/bounding_box3_d__functions.h"
#include "zed_interfaces/msg/detail/bounding_box3_d__struct.h"


// Include directives for member types
// Member `corners`
#include "zed_interfaces/msg/keypoint3_d.h"
// Member `corners`
#include "zed_interfaces/msg/detail/keypoint3_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zed_interfaces__msg__BoundingBox3D__init(message_memory);
}

void BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_fini_function(void * message_memory)
{
  zed_interfaces__msg__BoundingBox3D__fini(message_memory);
}

size_t BoundingBox3D__rosidl_typesupport_introspection_c__size_function__Keypoint3D__corners(
  const void * untyped_member)
{
  (void)untyped_member;
  return 8;
}

const void * BoundingBox3D__rosidl_typesupport_introspection_c__get_const_function__Keypoint3D__corners(
  const void * untyped_member, size_t index)
{
  const zed_interfaces__msg__Keypoint3D ** member =
    (const zed_interfaces__msg__Keypoint3D **)(untyped_member);
  return &(*member)[index];
}

void * BoundingBox3D__rosidl_typesupport_introspection_c__get_function__Keypoint3D__corners(
  void * untyped_member, size_t index)
{
  zed_interfaces__msg__Keypoint3D ** member =
    (zed_interfaces__msg__Keypoint3D **)(untyped_member);
  return &(*member)[index];
}

static rosidl_typesupport_introspection_c__MessageMember BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_message_member_array[1] = {
  {
    "corners",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces__msg__BoundingBox3D, corners),  // bytes offset in struct
    NULL,  // default value
    BoundingBox3D__rosidl_typesupport_introspection_c__size_function__Keypoint3D__corners,  // size() function pointer
    BoundingBox3D__rosidl_typesupport_introspection_c__get_const_function__Keypoint3D__corners,  // get_const(index) function pointer
    BoundingBox3D__rosidl_typesupport_introspection_c__get_function__Keypoint3D__corners,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_message_members = {
  "zed_interfaces__msg",  // message namespace
  "BoundingBox3D",  // message name
  1,  // number of fields
  sizeof(zed_interfaces__msg__BoundingBox3D),
  BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_message_member_array,  // message members
  BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_init_function,  // function to initialize message memory (memory has to be allocated)
  BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_message_type_support_handle = {
  0,
  &BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zed_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, msg, BoundingBox3D)() {
  BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, msg, Keypoint3D)();
  if (!BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_message_type_support_handle.typesupport_identifier) {
    BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BoundingBox3D__rosidl_typesupport_introspection_c__BoundingBox3D_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
