// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface:msg/Count.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__COUNT__TRAITS_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__COUNT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface/msg/detail/count__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Count & msg,
  std::ostream & out)
{
  out << "{";
  // member: end_count
  {
    out << "end_count: ";
    rosidl_generator_traits::value_to_yaml(msg.end_count, out);
    out << ", ";
  }

  // member: current_count
  {
    out << "current_count: ";
    rosidl_generator_traits::value_to_yaml(msg.current_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Count & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: end_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_count: ";
    rosidl_generator_traits::value_to_yaml(msg.end_count, out);
    out << "\n";
  }

  // member: current_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_count: ";
    rosidl_generator_traits::value_to_yaml(msg.current_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Count & msg, bool use_flow_style = false)
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
  const custom_interface::msg::Count & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::msg::Count & msg)
{
  return custom_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::msg::Count>()
{
  return "custom_interface::msg::Count";
}

template<>
inline const char * name<custom_interface::msg::Count>()
{
  return "custom_interface/msg/Count";
}

template<>
struct has_fixed_size<custom_interface::msg::Count>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interface::msg::Count>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interface::msg::Count>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__COUNT__TRAITS_HPP_
