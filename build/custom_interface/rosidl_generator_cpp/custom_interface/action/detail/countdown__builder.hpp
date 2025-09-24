// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:action/Countdown.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__ACTION__DETAIL__COUNTDOWN__BUILDER_HPP_
#define CUSTOM_INTERFACE__ACTION__DETAIL__COUNTDOWN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/action/detail/countdown__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace action
{

namespace builder
{

class Init_Countdown_Goal_start_from
{
public:
  Init_Countdown_Goal_start_from()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface::action::Countdown_Goal start_from(::custom_interface::action::Countdown_Goal::_start_from_type arg)
  {
    msg_.start_from = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::action::Countdown_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::action::Countdown_Goal>()
{
  return custom_interface::action::builder::Init_Countdown_Goal_start_from();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace action
{

namespace builder
{

class Init_Countdown_Result_result_text
{
public:
  Init_Countdown_Result_result_text()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface::action::Countdown_Result result_text(::custom_interface::action::Countdown_Result::_result_text_type arg)
  {
    msg_.result_text = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::action::Countdown_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::action::Countdown_Result>()
{
  return custom_interface::action::builder::Init_Countdown_Result_result_text();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace action
{

namespace builder
{

class Init_Countdown_Feedback_current
{
public:
  Init_Countdown_Feedback_current()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface::action::Countdown_Feedback current(::custom_interface::action::Countdown_Feedback::_current_type arg)
  {
    msg_.current = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::action::Countdown_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::action::Countdown_Feedback>()
{
  return custom_interface::action::builder::Init_Countdown_Feedback_current();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace action
{

namespace builder
{

class Init_Countdown_SendGoal_Request_goal
{
public:
  explicit Init_Countdown_SendGoal_Request_goal(::custom_interface::action::Countdown_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::custom_interface::action::Countdown_SendGoal_Request goal(::custom_interface::action::Countdown_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::action::Countdown_SendGoal_Request msg_;
};

class Init_Countdown_SendGoal_Request_goal_id
{
public:
  Init_Countdown_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Countdown_SendGoal_Request_goal goal_id(::custom_interface::action::Countdown_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Countdown_SendGoal_Request_goal(msg_);
  }

private:
  ::custom_interface::action::Countdown_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::action::Countdown_SendGoal_Request>()
{
  return custom_interface::action::builder::Init_Countdown_SendGoal_Request_goal_id();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace action
{

namespace builder
{

class Init_Countdown_SendGoal_Response_stamp
{
public:
  explicit Init_Countdown_SendGoal_Response_stamp(::custom_interface::action::Countdown_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::custom_interface::action::Countdown_SendGoal_Response stamp(::custom_interface::action::Countdown_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::action::Countdown_SendGoal_Response msg_;
};

class Init_Countdown_SendGoal_Response_accepted
{
public:
  Init_Countdown_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Countdown_SendGoal_Response_stamp accepted(::custom_interface::action::Countdown_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Countdown_SendGoal_Response_stamp(msg_);
  }

private:
  ::custom_interface::action::Countdown_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::action::Countdown_SendGoal_Response>()
{
  return custom_interface::action::builder::Init_Countdown_SendGoal_Response_accepted();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace action
{

namespace builder
{

class Init_Countdown_GetResult_Request_goal_id
{
public:
  Init_Countdown_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface::action::Countdown_GetResult_Request goal_id(::custom_interface::action::Countdown_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::action::Countdown_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::action::Countdown_GetResult_Request>()
{
  return custom_interface::action::builder::Init_Countdown_GetResult_Request_goal_id();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace action
{

namespace builder
{

class Init_Countdown_GetResult_Response_result
{
public:
  explicit Init_Countdown_GetResult_Response_result(::custom_interface::action::Countdown_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::custom_interface::action::Countdown_GetResult_Response result(::custom_interface::action::Countdown_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::action::Countdown_GetResult_Response msg_;
};

class Init_Countdown_GetResult_Response_status
{
public:
  Init_Countdown_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Countdown_GetResult_Response_result status(::custom_interface::action::Countdown_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Countdown_GetResult_Response_result(msg_);
  }

private:
  ::custom_interface::action::Countdown_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::action::Countdown_GetResult_Response>()
{
  return custom_interface::action::builder::Init_Countdown_GetResult_Response_status();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace action
{

namespace builder
{

class Init_Countdown_FeedbackMessage_feedback
{
public:
  explicit Init_Countdown_FeedbackMessage_feedback(::custom_interface::action::Countdown_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::custom_interface::action::Countdown_FeedbackMessage feedback(::custom_interface::action::Countdown_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::action::Countdown_FeedbackMessage msg_;
};

class Init_Countdown_FeedbackMessage_goal_id
{
public:
  Init_Countdown_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Countdown_FeedbackMessage_feedback goal_id(::custom_interface::action::Countdown_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Countdown_FeedbackMessage_feedback(msg_);
  }

private:
  ::custom_interface::action::Countdown_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::action::Countdown_FeedbackMessage>()
{
  return custom_interface::action::builder::Init_Countdown_FeedbackMessage_goal_id();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__ACTION__DETAIL__COUNTDOWN__BUILDER_HPP_
