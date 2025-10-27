// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:srv/CalculateDistance.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__CALCULATE_DISTANCE__STRUCT_H_
#define CUSTOM_INTERFACE__SRV__DETAIL__CALCULATE_DISTANCE__STRUCT_H_

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

/// Struct defined in srv/CalculateDistance in the package custom_interface.
typedef struct custom_interface__srv__CalculateDistance_Request
{
  geometry_msgs__msg__Point start_point;
  geometry_msgs__msg__Point end_point;
} custom_interface__srv__CalculateDistance_Request;

// Struct for a sequence of custom_interface__srv__CalculateDistance_Request.
typedef struct custom_interface__srv__CalculateDistance_Request__Sequence
{
  custom_interface__srv__CalculateDistance_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__CalculateDistance_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CalculateDistance in the package custom_interface.
typedef struct custom_interface__srv__CalculateDistance_Response
{
  float distance;
  bool success;
  rosidl_runtime_c__String message;
} custom_interface__srv__CalculateDistance_Response;

// Struct for a sequence of custom_interface__srv__CalculateDistance_Response.
typedef struct custom_interface__srv__CalculateDistance_Response__Sequence
{
  custom_interface__srv__CalculateDistance_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__CalculateDistance_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__CALCULATE_DISTANCE__STRUCT_H_
