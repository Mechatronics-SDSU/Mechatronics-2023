// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from zed_interfaces:srv/StartSvoRec.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/srv/detail/start_svo_rec__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "zed_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zed_interfaces/srv/detail/start_svo_rec__struct.h"
#include "zed_interfaces/srv/detail/start_svo_rec__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // svo_filename
#include "rosidl_runtime_c/string_functions.h"  // svo_filename

// forward declare type support functions


using _StartSvoRec_Request__ros_msg_type = zed_interfaces__srv__StartSvoRec_Request;

static bool _StartSvoRec_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _StartSvoRec_Request__ros_msg_type * ros_message = static_cast<const _StartSvoRec_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: bitrate
  {
    cdr << ros_message->bitrate;
  }

  // Field name: compression_mode
  {
    cdr << ros_message->compression_mode;
  }

  // Field name: target_framerate
  {
    cdr << ros_message->target_framerate;
  }

  // Field name: input_transcode
  {
    cdr << (ros_message->input_transcode ? true : false);
  }

  // Field name: svo_filename
  {
    const rosidl_runtime_c__String * str = &ros_message->svo_filename;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _StartSvoRec_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _StartSvoRec_Request__ros_msg_type * ros_message = static_cast<_StartSvoRec_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: bitrate
  {
    cdr >> ros_message->bitrate;
  }

  // Field name: compression_mode
  {
    cdr >> ros_message->compression_mode;
  }

  // Field name: target_framerate
  {
    cdr >> ros_message->target_framerate;
  }

  // Field name: input_transcode
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->input_transcode = tmp ? true : false;
  }

  // Field name: svo_filename
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->svo_filename.data) {
      rosidl_runtime_c__String__init(&ros_message->svo_filename);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->svo_filename,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'svo_filename'\n");
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t get_serialized_size_zed_interfaces__srv__StartSvoRec_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _StartSvoRec_Request__ros_msg_type * ros_message = static_cast<const _StartSvoRec_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name bitrate
  {
    size_t item_size = sizeof(ros_message->bitrate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name compression_mode
  {
    size_t item_size = sizeof(ros_message->compression_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_framerate
  {
    size_t item_size = sizeof(ros_message->target_framerate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name input_transcode
  {
    size_t item_size = sizeof(ros_message->input_transcode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name svo_filename
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->svo_filename.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _StartSvoRec_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_zed_interfaces__srv__StartSvoRec_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t max_serialized_size_zed_interfaces__srv__StartSvoRec_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: bitrate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: compression_mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: target_framerate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: input_transcode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: svo_filename
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _StartSvoRec_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_zed_interfaces__srv__StartSvoRec_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_StartSvoRec_Request = {
  "zed_interfaces::srv",
  "StartSvoRec_Request",
  _StartSvoRec_Request__cdr_serialize,
  _StartSvoRec_Request__cdr_deserialize,
  _StartSvoRec_Request__get_serialized_size,
  _StartSvoRec_Request__max_serialized_size
};

static rosidl_message_type_support_t _StartSvoRec_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_StartSvoRec_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, srv, StartSvoRec_Request)() {
  return &_StartSvoRec_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "zed_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "zed_interfaces/srv/detail/start_svo_rec__struct.h"
// already included above
// #include "zed_interfaces/srv/detail/start_svo_rec__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/string.h"  // message
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions


using _StartSvoRec_Response__ros_msg_type = zed_interfaces__srv__StartSvoRec_Response;

static bool _StartSvoRec_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _StartSvoRec_Response__ros_msg_type * ros_message = static_cast<const _StartSvoRec_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _StartSvoRec_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _StartSvoRec_Response__ros_msg_type * ros_message = static_cast<_StartSvoRec_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t get_serialized_size_zed_interfaces__srv__StartSvoRec_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _StartSvoRec_Response__ros_msg_type * ros_message = static_cast<const _StartSvoRec_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _StartSvoRec_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_zed_interfaces__srv__StartSvoRec_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_zed_interfaces
size_t max_serialized_size_zed_interfaces__srv__StartSvoRec_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: success
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _StartSvoRec_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_zed_interfaces__srv__StartSvoRec_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_StartSvoRec_Response = {
  "zed_interfaces::srv",
  "StartSvoRec_Response",
  _StartSvoRec_Response__cdr_serialize,
  _StartSvoRec_Response__cdr_deserialize,
  _StartSvoRec_Response__get_serialized_size,
  _StartSvoRec_Response__max_serialized_size
};

static rosidl_message_type_support_t _StartSvoRec_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_StartSvoRec_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, srv, StartSvoRec_Response)() {
  return &_StartSvoRec_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "zed_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "zed_interfaces/srv/start_svo_rec.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t StartSvoRec__callbacks = {
  "zed_interfaces::srv",
  "StartSvoRec",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, srv, StartSvoRec_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, srv, StartSvoRec_Response)(),
};

static rosidl_service_type_support_t StartSvoRec__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &StartSvoRec__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, zed_interfaces, srv, StartSvoRec)() {
  return &StartSvoRec__handle;
}

#if defined(__cplusplus)
}
#endif
