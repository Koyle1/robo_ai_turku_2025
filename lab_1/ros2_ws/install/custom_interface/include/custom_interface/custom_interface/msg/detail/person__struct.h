// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:msg/Person.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__PERSON__STRUCT_H_
#define CUSTOM_INTERFACE__MSG__DETAIL__PERSON__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Person in the package custom_interface.
typedef struct custom_interface__msg__Person
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String name;
  int32_t age;
  bool is_student;
} custom_interface__msg__Person;

// Struct for a sequence of custom_interface__msg__Person.
typedef struct custom_interface__msg__Person__Sequence
{
  custom_interface__msg__Person * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__msg__Person__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__PERSON__STRUCT_H_
