// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface:msg/ResponseDistanceMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__RESPONSE_DISTANCE_MSG__TRAITS_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__RESPONSE_DISTANCE_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface/msg/detail/response_distance_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const ResponseDistanceMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: place_holder
  {
    out << "place_holder: ";
    rosidl_generator_traits::value_to_yaml(msg.place_holder, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ResponseDistanceMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: place_holder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "place_holder: ";
    rosidl_generator_traits::value_to_yaml(msg.place_holder, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ResponseDistanceMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_interface

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface::msg::ResponseDistanceMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::msg::ResponseDistanceMsg & msg)
{
  return custom_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::msg::ResponseDistanceMsg>()
{
  return "custom_interface::msg::ResponseDistanceMsg";
}

template<>
inline const char * name<custom_interface::msg::ResponseDistanceMsg>()
{
  return "custom_interface/msg/ResponseDistanceMsg";
}

template<>
struct has_fixed_size<custom_interface::msg::ResponseDistanceMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interface::msg::ResponseDistanceMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interface::msg::ResponseDistanceMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__RESPONSE_DISTANCE_MSG__TRAITS_HPP_
