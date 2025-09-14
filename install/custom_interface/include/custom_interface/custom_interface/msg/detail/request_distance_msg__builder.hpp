// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:msg/RequestDistanceMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__BUILDER_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/msg/detail/request_distance_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace msg
{

namespace builder
{

class Init_RequestDistanceMsg_end_point
{
public:
  explicit Init_RequestDistanceMsg_end_point(::custom_interface::msg::RequestDistanceMsg & msg)
  : msg_(msg)
  {}
  ::custom_interface::msg::RequestDistanceMsg end_point(::custom_interface::msg::RequestDistanceMsg::_end_point_type arg)
  {
    msg_.end_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::msg::RequestDistanceMsg msg_;
};

class Init_RequestDistanceMsg_start_point
{
public:
  Init_RequestDistanceMsg_start_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RequestDistanceMsg_end_point start_point(::custom_interface::msg::RequestDistanceMsg::_start_point_type arg)
  {
    msg_.start_point = std::move(arg);
    return Init_RequestDistanceMsg_end_point(msg_);
  }

private:
  ::custom_interface::msg::RequestDistanceMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::msg::RequestDistanceMsg>()
{
  return custom_interface::msg::builder::Init_RequestDistanceMsg_start_point();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__BUILDER_HPP_
