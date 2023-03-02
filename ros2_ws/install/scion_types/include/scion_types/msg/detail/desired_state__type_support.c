// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from scion_types:msg/DesiredState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "scion_types/msg/detail/desired_state__rosidl_typesupport_introspection_c.h"
#include "scion_types/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "scion_types/msg/detail/desired_state__functions.h"
#include "scion_types/msg/detail/desired_state__struct.h"


// Include directives for member types
// Member `desired_state`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void DesiredState__rosidl_typesupport_introspection_c__DesiredState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scion_types__msg__DesiredState__init(message_memory);
}

void DesiredState__rosidl_typesupport_introspection_c__DesiredState_fini_function(void * message_memory)
{
  scion_types__msg__DesiredState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember DesiredState__rosidl_typesupport_introspection_c__DesiredState_message_member_array[1] = {
  {
    "desired_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scion_types__msg__DesiredState, desired_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers DesiredState__rosidl_typesupport_introspection_c__DesiredState_message_members = {
  "scion_types__msg",  // message namespace
  "DesiredState",  // message name
  1,  // number of fields
  sizeof(scion_types__msg__DesiredState),
  DesiredState__rosidl_typesupport_introspection_c__DesiredState_message_member_array,  // message members
  DesiredState__rosidl_typesupport_introspection_c__DesiredState_init_function,  // function to initialize message memory (memory has to be allocated)
  DesiredState__rosidl_typesupport_introspection_c__DesiredState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t DesiredState__rosidl_typesupport_introspection_c__DesiredState_message_type_support_handle = {
  0,
  &DesiredState__rosidl_typesupport_introspection_c__DesiredState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scion_types
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scion_types, msg, DesiredState)() {
  if (!DesiredState__rosidl_typesupport_introspection_c__DesiredState_message_type_support_handle.typesupport_identifier) {
    DesiredState__rosidl_typesupport_introspection_c__DesiredState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &DesiredState__rosidl_typesupport_introspection_c__DesiredState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
