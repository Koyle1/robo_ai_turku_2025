// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interface:msg/Count.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__COUNT__STRUCT_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__COUNT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interface__msg__Count __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface__msg__Count __declspec(deprecated)
#endif

namespace custom_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Count_
{
  using Type = Count_<ContainerAllocator>;

  explicit Count_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->end_count = 0l;
      this->current_count = 0l;
    }
  }

  explicit Count_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->end_count = 0l;
      this->current_count = 0l;
    }
  }

  // field types and members
  using _end_count_type =
    int32_t;
  _end_count_type end_count;
  using _current_count_type =
    int32_t;
  _current_count_type current_count;

  // setters for named parameter idiom
  Type & set__end_count(
    const int32_t & _arg)
  {
    this->end_count = _arg;
    return *this;
  }
  Type & set__current_count(
    const int32_t & _arg)
  {
    this->current_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface::msg::Count_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface::msg::Count_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface::msg::Count_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface::msg::Count_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface::msg::Count_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface::msg::Count_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface::msg::Count_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface::msg::Count_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface::msg::Count_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface::msg::Count_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface__msg__Count
    std::shared_ptr<custom_interface::msg::Count_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface__msg__Count
    std::shared_ptr<custom_interface::msg::Count_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Count_ & other) const
  {
    if (this->end_count != other.end_count) {
      return false;
    }
    if (this->current_count != other.current_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const Count_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Count_

// alias to use template instance with default allocator
using Count =
  custom_interface::msg::Count_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__COUNT__STRUCT_HPP_
