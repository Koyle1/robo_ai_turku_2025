// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface:srv/CancelRequest.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__CANCEL_REQUEST__TRAITS_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__CANCEL_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface/srv/detail/cancel_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const CancelRequest_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: countdown_id
  {
    out << "countdown_id: ";
    rosidl_generator_traits::value_to_yaml(msg.countdown_id, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CancelRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: countdown_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "countdown_id: ";
    rosidl_generator_traits::value_to_yaml(msg.countdown_id, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CancelRequest_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interface

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface::srv::CancelRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::srv::CancelRequest_Request & msg)
{
  return custom_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::srv::CancelRequest_Request>()
{
  return "custom_interface::srv::CancelRequest_Request";
}

template<>
inline const char * name<custom_interface::srv::CancelRequest_Request>()
{
  return "custom_interface/srv/CancelRequest_Request";
}

template<>
struct has_fixed_size<custom_interface::srv::CancelRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interface::srv::CancelRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interface::srv::CancelRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const CancelRequest_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: cancel_delay
  {
    out << "cancel_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.cancel_delay, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CancelRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: cancel_delay
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cancel_delay: ";
    rosidl_generator_traits::value_to_yaml(msg.cancel_delay, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CancelRequest_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interface

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface::srv::CancelRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::srv::CancelRequest_Response & msg)
{
  return custom_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::srv::CancelRequest_Response>()
{
  return "custom_interface::srv::CancelRequest_Response";
}

template<>
inline const char * name<custom_interface::srv::CancelRequest_Response>()
{
  return "custom_interface/srv/CancelRequest_Response";
}

template<>
struct has_fixed_size<custom_interface::srv::CancelRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interface::srv::CancelRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interface::srv::CancelRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interface::srv::CancelRequest>()
{
  return "custom_interface::srv::CancelRequest";
}

template<>
inline const char * name<custom_interface::srv::CancelRequest>()
{
  return "custom_interface/srv/CancelRequest";
}

template<>
struct has_fixed_size<custom_interface::srv::CancelRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interface::srv::CancelRequest_Request>::value &&
    has_fixed_size<custom_interface::srv::CancelRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interface::srv::CancelRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interface::srv::CancelRequest_Request>::value &&
    has_bounded_size<custom_interface::srv::CancelRequest_Response>::value
  >
{
};

template<>
struct is_service<custom_interface::srv::CancelRequest>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interface::srv::CancelRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interface::srv::CancelRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__CANCEL_REQUEST__TRAITS_HPP_
