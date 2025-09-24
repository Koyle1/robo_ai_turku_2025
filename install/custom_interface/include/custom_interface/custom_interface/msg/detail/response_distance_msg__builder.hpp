// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:msg/ResponseDistanceMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__RESPONSE_DISTANCE_MSG__BUILDER_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__RESPONSE_DISTANCE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/msg/detail/response_distance_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace msg
{

namespace builder
{

class Init_ResponseDistanceMsg_place_holder
{
public:
  Init_ResponseDistanceMsg_place_holder()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface::msg::ResponseDistanceMsg place_holder(::custom_interface::msg::ResponseDistanceMsg::_place_holder_type arg)
  {
    msg_.place_holder = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::msg::ResponseDistanceMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::msg::ResponseDistanceMsg>()
{
  return custom_interface::msg::builder::Init_ResponseDistanceMsg_place_holder();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__RESPONSE_DISTANCE_MSG__BUILDER_HPP_
