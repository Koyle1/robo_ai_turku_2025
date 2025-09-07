// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:srv/CalculateDistance.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__CALCULATE_DISTANCE__BUILDER_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__CALCULATE_DISTANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/srv/detail/calculate_distance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_CalculateDistance_Request_p2
{
public:
  explicit Init_CalculateDistance_Request_p2(::custom_interface::srv::CalculateDistance_Request & msg)
  : msg_(msg)
  {}
  ::custom_interface::srv::CalculateDistance_Request p2(::custom_interface::srv::CalculateDistance_Request::_p2_type arg)
  {
    msg_.p2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::CalculateDistance_Request msg_;
};

class Init_CalculateDistance_Request_p1
{
public:
  Init_CalculateDistance_Request_p1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalculateDistance_Request_p2 p1(::custom_interface::srv::CalculateDistance_Request::_p1_type arg)
  {
    msg_.p1 = std::move(arg);
    return Init_CalculateDistance_Request_p2(msg_);
  }

private:
  ::custom_interface::srv::CalculateDistance_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::CalculateDistance_Request>()
{
  return custom_interface::srv::builder::Init_CalculateDistance_Request_p1();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_CalculateDistance_Response_message
{
public:
  explicit Init_CalculateDistance_Response_message(::custom_interface::srv::CalculateDistance_Response & msg)
  : msg_(msg)
  {}
  ::custom_interface::srv::CalculateDistance_Response message(::custom_interface::srv::CalculateDistance_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::CalculateDistance_Response msg_;
};

class Init_CalculateDistance_Response_success
{
public:
  explicit Init_CalculateDistance_Response_success(::custom_interface::srv::CalculateDistance_Response & msg)
  : msg_(msg)
  {}
  Init_CalculateDistance_Response_message success(::custom_interface::srv::CalculateDistance_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CalculateDistance_Response_message(msg_);
  }

private:
  ::custom_interface::srv::CalculateDistance_Response msg_;
};

class Init_CalculateDistance_Response_distance
{
public:
  Init_CalculateDistance_Response_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalculateDistance_Response_success distance(::custom_interface::srv::CalculateDistance_Response::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_CalculateDistance_Response_success(msg_);
  }

private:
  ::custom_interface::srv::CalculateDistance_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::CalculateDistance_Response>()
{
  return custom_interface::srv::builder::Init_CalculateDistance_Response_distance();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__CALCULATE_DISTANCE__BUILDER_HPP_
