// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:msg/Person.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__PERSON__BUILDER_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__PERSON__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/msg/detail/person__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace msg
{

namespace builder
{

class Init_Person_is_student
{
public:
  explicit Init_Person_is_student(::custom_interface::msg::Person & msg)
  : msg_(msg)
  {}
  ::custom_interface::msg::Person is_student(::custom_interface::msg::Person::_is_student_type arg)
  {
    msg_.is_student = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::msg::Person msg_;
};

class Init_Person_age
{
public:
  explicit Init_Person_age(::custom_interface::msg::Person & msg)
  : msg_(msg)
  {}
  Init_Person_is_student age(::custom_interface::msg::Person::_age_type arg)
  {
    msg_.age = std::move(arg);
    return Init_Person_is_student(msg_);
  }

private:
  ::custom_interface::msg::Person msg_;
};

class Init_Person_name
{
public:
  explicit Init_Person_name(::custom_interface::msg::Person & msg)
  : msg_(msg)
  {}
  Init_Person_age name(::custom_interface::msg::Person::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Person_age(msg_);
  }

private:
  ::custom_interface::msg::Person msg_;
};

class Init_Person_header
{
public:
  Init_Person_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Person_name header(::custom_interface::msg::Person::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Person_name(msg_);
  }

private:
  ::custom_interface::msg::Person msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::msg::Person>()
{
  return custom_interface::msg::builder::Init_Person_header();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__PERSON__BUILDER_HPP_
