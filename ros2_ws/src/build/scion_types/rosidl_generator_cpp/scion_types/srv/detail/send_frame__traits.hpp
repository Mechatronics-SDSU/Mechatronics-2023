// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from scion_types:srv/SendFrame.idl
// generated code does not contain a copyright notice

#ifndef SCION_TYPES__SRV__DETAIL__SEND_FRAME__TRAITS_HPP_
#define SCION_TYPES__SRV__DETAIL__SEND_FRAME__TRAITS_HPP_

#include "scion_types/srv/detail/send_frame__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<scion_types::srv::SendFrame_Request>()
{
  return "scion_types::srv::SendFrame_Request";
}

template<>
inline const char * name<scion_types::srv::SendFrame_Request>()
{
  return "scion_types/srv/SendFrame_Request";
}

template<>
struct has_fixed_size<scion_types::srv::SendFrame_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<scion_types::srv::SendFrame_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<scion_types::srv::SendFrame_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<scion_types::srv::SendFrame_Response>()
{
  return "scion_types::srv::SendFrame_Response";
}

template<>
inline const char * name<scion_types::srv::SendFrame_Response>()
{
  return "scion_types/srv/SendFrame_Response";
}

template<>
struct has_fixed_size<scion_types::srv::SendFrame_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<scion_types::srv::SendFrame_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<scion_types::srv::SendFrame_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<scion_types::srv::SendFrame>()
{
  return "scion_types::srv::SendFrame";
}

template<>
inline const char * name<scion_types::srv::SendFrame>()
{
  return "scion_types/srv/SendFrame";
}

template<>
struct has_fixed_size<scion_types::srv::SendFrame>
  : std::integral_constant<
    bool,
    has_fixed_size<scion_types::srv::SendFrame_Request>::value &&
    has_fixed_size<scion_types::srv::SendFrame_Response>::value
  >
{
};

template<>
struct has_bounded_size<scion_types::srv::SendFrame>
  : std::integral_constant<
    bool,
    has_bounded_size<scion_types::srv::SendFrame_Request>::value &&
    has_bounded_size<scion_types::srv::SendFrame_Response>::value
  >
{
};

template<>
struct is_service<scion_types::srv::SendFrame>
  : std::true_type
{
};

template<>
struct is_service_request<scion_types::srv::SendFrame_Request>
  : std::true_type
{
};

template<>
struct is_service_response<scion_types::srv::SendFrame_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SCION_TYPES__SRV__DETAIL__SEND_FRAME__TRAITS_HPP_
