// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from zed_interfaces:msg/ObjectsStamped.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "zed_interfaces/msg/detail/objects_stamped__struct.hpp"
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

void ObjectsStamped_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) zed_interfaces::msg::ObjectsStamped(_init);
}

void ObjectsStamped_fini_function(void * message_memory)
{
  auto typed_message = static_cast<zed_interfaces::msg::ObjectsStamped *>(message_memory);
  typed_message->~ObjectsStamped();
}

size_t size_function__ObjectsStamped__objects(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<zed_interfaces::msg::Object> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ObjectsStamped__objects(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<zed_interfaces::msg::Object> *>(untyped_member);
  return &member[index];
}

void * get_function__ObjectsStamped__objects(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<zed_interfaces::msg::Object> *>(untyped_member);
  return &member[index];
}

void resize_function__ObjectsStamped__objects(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<zed_interfaces::msg::Object> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ObjectsStamped_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces::msg::ObjectsStamped, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<zed_interfaces::msg::Object>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zed_interfaces::msg::ObjectsStamped, objects),  // bytes offset in struct
    nullptr,  // default value
    size_function__ObjectsStamped__objects,  // size() function pointer
    get_const_function__ObjectsStamped__objects,  // get_const(index) function pointer
    get_function__ObjectsStamped__objects,  // get(index) function pointer
    resize_function__ObjectsStamped__objects  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ObjectsStamped_message_members = {
  "zed_interfaces::msg",  // message namespace
  "ObjectsStamped",  // message name
  2,  // number of fields
  sizeof(zed_interfaces::msg::ObjectsStamped),
  ObjectsStamped_message_member_array,  // message members
  ObjectsStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  ObjectsStamped_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ObjectsStamped_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ObjectsStamped_message_members,
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
get_message_type_support_handle<zed_interfaces::msg::ObjectsStamped>()
{
  return &::zed_interfaces::msg::rosidl_typesupport_introspection_cpp::ObjectsStamped_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, zed_interfaces, msg, ObjectsStamped)() {
  return &::zed_interfaces::msg::rosidl_typesupport_introspection_cpp::ObjectsStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
