// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:srv/CancelRequest.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__CANCEL_REQUEST__STRUCT_H_
#define CUSTOM_INTERFACE__SRV__DETAIL__CANCEL_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CancelRequest in the package custom_interface.
typedef struct custom_interface__srv__CancelRequest_Request
{
  int32_t countdown_id;
  rosidl_runtime_c__String reason;
} custom_interface__srv__CancelRequest_Request;

// Struct for a sequence of custom_interface__srv__CancelRequest_Request.
typedef struct custom_interface__srv__CancelRequest_Request__Sequence
{
  custom_interface__srv__CancelRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__CancelRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CancelRequest in the package custom_interface.
typedef struct custom_interface__srv__CancelRequest_Response
{
  bool success;
  rosidl_runtime_c__String message;
  double cancel_delay;
} custom_interface__srv__CancelRequest_Response;

// Struct for a sequence of custom_interface__srv__CancelRequest_Response.
typedef struct custom_interface__srv__CancelRequest_Response__Sequence
{
  custom_interface__srv__CancelRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__CancelRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__CANCEL_REQUEST__STRUCT_H_
