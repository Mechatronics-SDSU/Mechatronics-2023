// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from scion_types:msg/DesiredState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "scion_types/msg/detail/desired_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace scion_types
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DesiredState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) scion_types::msg::DesiredState(_init);
}

void DesiredState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<scion_types::msg::DesiredState *>(message_memory);
  typed_message->~DesiredState();
}

size_t size_function__DesiredState__desired_state(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DesiredState__desired_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__DesiredState__desired_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__DesiredState__desired_state(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DesiredState_message_member_array[1] = {
  {
    "desired_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scion_types::msg::DesiredState, desired_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__DesiredState__desired_state,  // size() function pointer
    get_const_function__DesiredState__desired_state,  // get_const(index) function pointer
    get_function__DesiredState__desired_state,  // get(index) function pointer
    resize_function__DesiredState__desired_state  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DesiredState_message_members = {
  "scion_types::msg",  // message namespace
  "DesiredState",  // message name
  1,  // number of fields
  sizeof(scion_types::msg::DesiredState),
  DesiredState_message_member_array,  // message members
  DesiredState_init_function,  // function to initialize message memory (memory has to be allocated)
  DesiredState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DesiredState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DesiredState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace scion_types


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<scion_types::msg::DesiredState>()
{
  return &::scion_types::msg::rosidl_typesupport_introspection_cpp::DesiredState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, scion_types, msg, DesiredState)() {
  return &::scion_types::msg::rosidl_typesupport_introspection_cpp::DesiredState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
