// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interface:msg/RequestDistanceMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__STRUCT_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'start_point'
// Member 'end_point'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interface__msg__RequestDistanceMsg __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface__msg__RequestDistanceMsg __declspec(deprecated)
#endif

namespace custom_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RequestDistanceMsg_
{
  using Type = RequestDistanceMsg_<ContainerAllocator>;

  explicit RequestDistanceMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_point(_init),
    end_point(_init)
  {
    (void)_init;
  }

  explicit RequestDistanceMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_point(_alloc, _init),
    end_point(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _start_point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _start_point_type start_point;
  using _end_point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _end_point_type end_point;

  // setters for named parameter idiom
  Type & set__start_point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->start_point = _arg;
    return *this;
  }
  Type & set__end_point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->end_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface::msg::RequestDistanceMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface::msg::RequestDistanceMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface::msg::RequestDistanceMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface::msg::RequestDistanceMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface::msg::RequestDistanceMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface::msg::RequestDistanceMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface::msg::RequestDistanceMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface::msg::RequestDistanceMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface::msg::RequestDistanceMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface::msg::RequestDistanceMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface__msg__RequestDistanceMsg
    std::shared_ptr<custom_interface::msg::RequestDistanceMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface__msg__RequestDistanceMsg
    std::shared_ptr<custom_interface::msg::RequestDistanceMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RequestDistanceMsg_ & other) const
  {
    if (this->start_point != other.start_point) {
      return false;
    }
    if (this->end_point != other.end_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const RequestDistanceMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RequestDistanceMsg_

// alias to use template instance with default allocator
using RequestDistanceMsg =
  custom_interface::msg::RequestDistanceMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__STRUCT_HPP_
