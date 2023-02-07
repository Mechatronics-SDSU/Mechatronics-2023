// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zed_interfaces:srv/StartSvoRec.idl
// generated code does not contain a copyright notice

#ifndef ZED_INTERFACES__SRV__DETAIL__START_SVO_REC__TRAITS_HPP_
#define ZED_INTERFACES__SRV__DETAIL__START_SVO_REC__TRAITS_HPP_

#include "zed_interfaces/srv/detail/start_svo_rec__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zed_interfaces::srv::StartSvoRec_Request>()
{
  return "zed_interfaces::srv::StartSvoRec_Request";
}

template<>
inline const char * name<zed_interfaces::srv::StartSvoRec_Request>()
{
  return "zed_interfaces/srv/StartSvoRec_Request";
}

template<>
struct has_fixed_size<zed_interfaces::srv::StartSvoRec_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<zed_interfaces::srv::StartSvoRec_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<zed_interfaces::srv::StartSvoRec_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zed_interfaces::srv::StartSvoRec_Response>()
{
  return "zed_interfaces::srv::StartSvoRec_Response";
}

template<>
inline const char * name<zed_interfaces::srv::StartSvoRec_Response>()
{
  return "zed_interfaces/srv/StartSvoRec_Response";
}

template<>
struct has_fixed_size<zed_interfaces::srv::StartSvoRec_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<zed_interfaces::srv::StartSvoRec_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<zed_interfaces::srv::StartSvoRec_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zed_interfaces::srv::StartSvoRec>()
{
  return "zed_interfaces::srv::StartSvoRec";
}

template<>
inline const char * name<zed_interfaces::srv::StartSvoRec>()
{
  return "zed_interfaces/srv/StartSvoRec";
}

template<>
struct has_fixed_size<zed_interfaces::srv::StartSvoRec>
  : std::integral_constant<
    bool,
    has_fixed_size<zed_interfaces::srv::StartSvoRec_Request>::value &&
    has_fixed_size<zed_interfaces::srv::StartSvoRec_Response>::value
  >
{
};

template<>
struct has_bounded_size<zed_interfaces::srv::StartSvoRec>
  : std::integral_constant<
    bool,
    has_bounded_size<zed_interfaces::srv::StartSvoRec_Request>::value &&
    has_bounded_size<zed_interfaces::srv::StartSvoRec_Response>::value
  >
{
};

template<>
struct is_service<zed_interfaces::srv::StartSvoRec>
  : std::true_type
{
};

template<>
struct is_service_request<zed_interfaces::srv::StartSvoRec_Request>
  : std::true_type
{
};

template<>
struct is_service_response<zed_interfaces::srv::StartSvoRec_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ZED_INTERFACES__SRV__DETAIL__START_SVO_REC__TRAITS_HPP_
