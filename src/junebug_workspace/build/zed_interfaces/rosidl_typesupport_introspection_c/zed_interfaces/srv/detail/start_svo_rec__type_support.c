// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from zed_interfaces:srv/StartSvoRec.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "zed_interfaces/srv/detail/start_svo_rec__rosidl_typesupport_introspection_c.h"
#include "zed_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "zed_interfaces/srv/detail/start_svo_rec__functions.h"
#include "zed_interfaces/srv/detail/start_svo_rec__struct.h"


// Include directives for member types
// Member `svo_filename`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zed_interfaces__srv__StartSvoRec_Request__init(message_memory);
}

void StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_fini_function(void * message_memory)
{
  zed_interfaces__srv__StartSvoRec_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_message_member_array[5] = {
  {
    "bitrate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces__srv__StartSvoRec_Request, bitrate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "compression_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces__srv__StartSvoRec_Request, compression_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_framerate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces__srv__StartSvoRec_Request, target_framerate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "input_transcode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces__srv__StartSvoRec_Request, input_transcode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "svo_filename",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces__srv__StartSvoRec_Request, svo_filename),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_message_members = {
  "zed_interfaces__srv",  // message namespace
  "StartSvoRec_Request",  // message name
  5,  // number of fields
  sizeof(zed_interfaces__srv__StartSvoRec_Request),
  StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_message_member_array,  // message members
  StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_message_type_support_handle = {
  0,
  &StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zed_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, srv, StartSvoRec_Request)() {
  if (!StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_message_type_support_handle.typesupport_identifier) {
    StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &StartSvoRec_Request__rosidl_typesupport_introspection_c__StartSvoRec_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "zed_interfaces/srv/detail/start_svo_rec__rosidl_typesupport_introspection_c.h"
// already included above
// #include "zed_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "zed_interfaces/srv/detail/start_svo_rec__functions.h"
// already included above
// #include "zed_interfaces/srv/detail/start_svo_rec__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zed_interfaces__srv__StartSvoRec_Response__init(message_memory);
}

void StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_fini_function(void * message_memory)
{
  zed_interfaces__srv__StartSvoRec_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces__srv__StartSvoRec_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces__srv__StartSvoRec_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_message_members = {
  "zed_interfaces__srv",  // message namespace
  "StartSvoRec_Response",  // message name
  2,  // number of fields
  sizeof(zed_interfaces__srv__StartSvoRec_Response),
  StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_message_member_array,  // message members
  StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_message_type_support_handle = {
  0,
  &StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zed_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, srv, StartSvoRec_Response)() {
  if (!StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_message_type_support_handle.typesupport_identifier) {
    StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &StartSvoRec_Response__rosidl_typesupport_introspection_c__StartSvoRec_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "zed_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "zed_interfaces/srv/detail/start_svo_rec__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers zed_interfaces__srv__detail__start_svo_rec__rosidl_typesupport_introspection_c__StartSvoRec_service_members = {
  "zed_interfaces__srv",  // service namespace
  "StartSvoRec",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // zed_interfaces__srv__detail__start_svo_rec__rosidl_typesupport_introspection_c__StartSvoRec_Request_message_type_support_handle,
  NULL  // response message
  // zed_interfaces__srv__detail__start_svo_rec__rosidl_typesupport_introspection_c__StartSvoRec_Response_message_type_support_handle
};

static rosidl_service_type_support_t zed_interfaces__srv__detail__start_svo_rec__rosidl_typesupport_introspection_c__StartSvoRec_service_type_support_handle = {
  0,
  &zed_interfaces__srv__detail__start_svo_rec__rosidl_typesupport_introspection_c__StartSvoRec_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, srv, StartSvoRec_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, srv, StartSvoRec_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zed_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, srv, StartSvoRec)() {
  if (!zed_interfaces__srv__detail__start_svo_rec__rosidl_typesupport_introspection_c__StartSvoRec_service_type_support_handle.typesupport_identifier) {
    zed_interfaces__srv__detail__start_svo_rec__rosidl_typesupport_introspection_c__StartSvoRec_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)zed_interfaces__srv__detail__start_svo_rec__rosidl_typesupport_introspection_c__StartSvoRec_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, srv, StartSvoRec_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zed_interfaces, srv, StartSvoRec_Response)()->data;
  }

  return &zed_interfaces__srv__detail__start_svo_rec__rosidl_typesupport_introspection_c__StartSvoRec_service_type_support_handle;
}
