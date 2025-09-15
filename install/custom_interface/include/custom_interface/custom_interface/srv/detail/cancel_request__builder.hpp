// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:srv/CancelRequest.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__CANCEL_REQUEST__BUILDER_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__CANCEL_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/srv/detail/cancel_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_CancelRequest_Request_reason
{
public:
  explicit Init_CancelRequest_Request_reason(::custom_interface::srv::CancelRequest_Request & msg)
  : msg_(msg)
  {}
  ::custom_interface::srv::CancelRequest_Request reason(::custom_interface::srv::CancelRequest_Request::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::CancelRequest_Request msg_;
};

class Init_CancelRequest_Request_countdown_id
{
public:
  Init_CancelRequest_Request_countdown_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CancelRequest_Request_reason countdown_id(::custom_interface::srv::CancelRequest_Request::_countdown_id_type arg)
  {
    msg_.countdown_id = std::move(arg);
    return Init_CancelRequest_Request_reason(msg_);
  }

private:
  ::custom_interface::srv::CancelRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::CancelRequest_Request>()
{
  return custom_interface::srv::builder::Init_CancelRequest_Request_countdown_id();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_CancelRequest_Response_cancel_delay
{
public:
  explicit Init_CancelRequest_Response_cancel_delay(::custom_interface::srv::CancelRequest_Response & msg)
  : msg_(msg)
  {}
  ::custom_interface::srv::CancelRequest_Response cancel_delay(::custom_interface::srv::CancelRequest_Response::_cancel_delay_type arg)
  {
    msg_.cancel_delay = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::CancelRequest_Response msg_;
};

class Init_CancelRequest_Response_message
{
public:
  explicit Init_CancelRequest_Response_message(::custom_interface::srv::CancelRequest_Response & msg)
  : msg_(msg)
  {}
  Init_CancelRequest_Response_cancel_delay message(::custom_interface::srv::CancelRequest_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_CancelRequest_Response_cancel_delay(msg_);
  }

private:
  ::custom_interface::srv::CancelRequest_Response msg_;
};

class Init_CancelRequest_Response_success
{
public:
  Init_CancelRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CancelRequest_Response_message success(::custom_interface::srv::CancelRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CancelRequest_Response_message(msg_);
  }

private:
  ::custom_interface::srv::CancelRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::CancelRequest_Response>()
{
  return custom_interface::srv::builder::Init_CancelRequest_Response_success();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__CANCEL_REQUEST__BUILDER_HPP_
