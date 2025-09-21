// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:msg/Count.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__COUNT__BUILDER_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__COUNT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/msg/detail/count__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace msg
{

namespace builder
{

class Init_Count_current_count
{
public:
  explicit Init_Count_current_count(::custom_interface::msg::Count & msg)
  : msg_(msg)
  {}
  ::custom_interface::msg::Count current_count(::custom_interface::msg::Count::_current_count_type arg)
  {
    msg_.current_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::msg::Count msg_;
};

class Init_Count_end_count
{
public:
  Init_Count_end_count()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Count_current_count end_count(::custom_interface::msg::Count::_end_count_type arg)
  {
    msg_.end_count = std::move(arg);
    return Init_Count_current_count(msg_);
  }

private:
  ::custom_interface::msg::Count msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::msg::Count>()
{
  return custom_interface::msg::builder::Init_Count_end_count();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__COUNT__BUILDER_HPP_
