// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from zed_interfaces:msg/BoundingBox3D.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "zed_interfaces/msg/detail/bounding_box3_d__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace zed_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BoundingBox3D_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) zed_interfaces::msg::BoundingBox3D(_init);
}

void BoundingBox3D_fini_function(void * message_memory)
{
  auto typed_message = static_cast<zed_interfaces::msg::BoundingBox3D *>(message_memory);
  typed_message->~BoundingBox3D();
}

size_t size_function__BoundingBox3D__corners(const void * untyped_member)
{
  (void)untyped_member;
  return 8;
}

const void * get_const_function__BoundingBox3D__corners(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<zed_interfaces::msg::Keypoint3D, 8> *>(untyped_member);
  return &member[index];
}

void * get_function__BoundingBox3D__corners(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<zed_interfaces::msg::Keypoint3D, 8> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BoundingBox3D_message_member_array[1] = {
  {
    "corners",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<zed_interfaces::msg::Keypoint3D>(),  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces::msg::BoundingBox3D, corners),  // bytes offset in struct
    nullptr,  // default value
    size_function__BoundingBox3D__corners,  // size() function pointer
    get_const_function__BoundingBox3D__corners,  // get_const(index) function pointer
    get_function__BoundingBox3D__corners,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BoundingBox3D_message_members = {
  "zed_interfaces::msg",  // message namespace
  "BoundingBox3D",  // message name
  1,  // number of fields
  sizeof(zed_interfaces::msg::BoundingBox3D),
  BoundingBox3D_message_member_array,  // message members
  BoundingBox3D_init_function,  // function to initialize message memory (memory has to be allocated)
  BoundingBox3D_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BoundingBox3D_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BoundingBox3D_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace zed_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<zed_interfaces::msg::BoundingBox3D>()
{
  return &::zed_interfaces::msg::rosidl_typesupport_introspection_cpp::BoundingBox3D_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, zed_interfaces, msg, BoundingBox3D)() {
  return &::zed_interfaces::msg::rosidl_typesupport_introspection_cpp::BoundingBox3D_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
