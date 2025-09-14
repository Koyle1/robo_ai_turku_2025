// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:msg/TaskMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__TASK_MSG__BUILDER_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__TASK_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/msg/detail/task_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace msg
{

namespace builder
{

class Init_TaskMsg_is_student
{
public:
  explicit Init_TaskMsg_is_student(::custom_interface::msg::TaskMsg & msg)
  : msg_(msg)
  {}
  ::custom_interface::msg::TaskMsg is_student(::custom_interface::msg::TaskMsg::_is_student_type arg)
  {
    msg_.is_student = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::msg::TaskMsg msg_;
};

class Init_TaskMsg_age
{
public:
  explicit Init_TaskMsg_age(::custom_interface::msg::TaskMsg & msg)
  : msg_(msg)
  {}
  Init_TaskMsg_is_student age(::custom_interface::msg::TaskMsg::_age_type arg)
  {
    msg_.age = std::move(arg);
    return Init_TaskMsg_is_student(msg_);
  }

private:
  ::custom_interface::msg::TaskMsg msg_;
};

class Init_TaskMsg_name
{
public:
  Init_TaskMsg_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TaskMsg_age name(::custom_interface::msg::TaskMsg::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_TaskMsg_age(msg_);
  }

private:
  ::custom_interface::msg::TaskMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::msg::TaskMsg>()
{
  return custom_interface::msg::builder::Init_TaskMsg_name();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__TASK_MSG__BUILDER_HPP_
