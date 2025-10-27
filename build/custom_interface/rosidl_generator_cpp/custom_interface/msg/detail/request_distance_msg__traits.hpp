// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface:msg/RequestDistanceMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__TRAITS_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface/msg/detail/request_distance_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'start_point'
// Member 'end_point'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace custom_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const RequestDistanceMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: start_point
  {
    out << "start_point: ";
    to_flow_style_yaml(msg.start_point, out);
    out << ", ";
  }

  // member: end_point
  {
    out << "end_point: ";
    to_flow_style_yaml(msg.end_point, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RequestDistanceMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_point:\n";
    to_block_style_yaml(msg.start_point, out, indentation + 2);
  }

  // member: end_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_point:\n";
    to_block_style_yaml(msg.end_point, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RequestDistanceMsg & msg, bool use_flow_style = false)
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
  const custom_interface::msg::RequestDistanceMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::msg::RequestDistanceMsg & msg)
{
  return custom_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::msg::RequestDistanceMsg>()
{
  return "custom_interface::msg::RequestDistanceMsg";
}

template<>
inline const char * name<custom_interface::msg::RequestDistanceMsg>()
{
  return "custom_interface/msg/RequestDistanceMsg";
}

template<>
struct has_fixed_size<custom_interface::msg::RequestDistanceMsg>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<custom_interface::msg::RequestDistanceMsg>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<custom_interface::msg::RequestDistanceMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__TRAITS_HPP_
