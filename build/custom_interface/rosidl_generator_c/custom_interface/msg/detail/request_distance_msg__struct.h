// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:msg/RequestDistanceMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__STRUCT_H_
#define CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start_point'
// Member 'end_point'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/RequestDistanceMsg in the package custom_interface.
typedef struct custom_interface__msg__RequestDistanceMsg
{
  geometry_msgs__msg__Point start_point;
  geometry_msgs__msg__Point end_point;
} custom_interface__msg__RequestDistanceMsg;

// Struct for a sequence of custom_interface__msg__RequestDistanceMsg.
typedef struct custom_interface__msg__RequestDistanceMsg__Sequence
{
  custom_interface__msg__RequestDistanceMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__msg__RequestDistanceMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__STRUCT_H_
