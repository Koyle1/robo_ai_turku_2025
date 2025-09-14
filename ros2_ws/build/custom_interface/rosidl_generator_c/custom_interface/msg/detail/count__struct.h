// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:msg/Count.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__COUNT__STRUCT_H_
#define CUSTOM_INTERFACE__MSG__DETAIL__COUNT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Count in the package custom_interface.
typedef struct custom_interface__msg__Count
{
  int32_t end_count;
  int32_t current_count;
} custom_interface__msg__Count;

// Struct for a sequence of custom_interface__msg__Count.
typedef struct custom_interface__msg__Count__Sequence
{
  custom_interface__msg__Count * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__msg__Count__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__COUNT__STRUCT_H_
