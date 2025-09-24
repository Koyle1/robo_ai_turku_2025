// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interface:msg/ResponseDistanceMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__RESPONSE_DISTANCE_MSG__STRUCT_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__RESPONSE_DISTANCE_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interface__msg__ResponseDistanceMsg __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface__msg__ResponseDistanceMsg __declspec(deprecated)
#endif

namespace custom_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ResponseDistanceMsg_
{
  using Type = ResponseDistanceMsg_<ContainerAllocator>;

  explicit ResponseDistanceMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->place_holder = "";
    }
  }

  explicit ResponseDistanceMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : place_holder(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->place_holder = "";
    }
  }

  // field types and members
  using _place_holder_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _place_holder_type place_holder;

  // setters for named parameter idiom
  Type & set__place_holder(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->place_holder = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface__msg__ResponseDistanceMsg
    std::shared_ptr<custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface__msg__ResponseDistanceMsg
    std::shared_ptr<custom_interface::msg::ResponseDistanceMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ResponseDistanceMsg_ & other) const
  {
    if (this->place_holder != other.place_holder) {
      return false;
    }
    return true;
  }
  bool operator!=(const ResponseDistanceMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ResponseDistanceMsg_

// alias to use template instance with default allocator
using ResponseDistanceMsg =
  custom_interface::msg::ResponseDistanceMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__RESPONSE_DISTANCE_MSG__STRUCT_HPP_
