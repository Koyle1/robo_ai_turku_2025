// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from custom_interface:msg/RequestDistanceMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__FUNCTIONS_H_
#define CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "custom_interface/msg/rosidl_generator_c__visibility_control.h"

#include "custom_interface/msg/detail/request_distance_msg__struct.h"

/// Initialize msg/RequestDistanceMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * custom_interface__msg__RequestDistanceMsg
 * )) before or use
 * custom_interface__msg__RequestDistanceMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
bool
custom_interface__msg__RequestDistanceMsg__init(custom_interface__msg__RequestDistanceMsg * msg);

/// Finalize msg/RequestDistanceMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
void
custom_interface__msg__RequestDistanceMsg__fini(custom_interface__msg__RequestDistanceMsg * msg);

/// Create msg/RequestDistanceMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * custom_interface__msg__RequestDistanceMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
custom_interface__msg__RequestDistanceMsg *
custom_interface__msg__RequestDistanceMsg__create();

/// Destroy msg/RequestDistanceMsg message.
/**
 * It calls
 * custom_interface__msg__RequestDistanceMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
void
custom_interface__msg__RequestDistanceMsg__destroy(custom_interface__msg__RequestDistanceMsg * msg);

/// Check for msg/RequestDistanceMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
bool
custom_interface__msg__RequestDistanceMsg__are_equal(const custom_interface__msg__RequestDistanceMsg * lhs, const custom_interface__msg__RequestDistanceMsg * rhs);

/// Copy a msg/RequestDistanceMsg message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
bool
custom_interface__msg__RequestDistanceMsg__copy(
  const custom_interface__msg__RequestDistanceMsg * input,
  custom_interface__msg__RequestDistanceMsg * output);

/// Initialize array of msg/RequestDistanceMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * custom_interface__msg__RequestDistanceMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
bool
custom_interface__msg__RequestDistanceMsg__Sequence__init(custom_interface__msg__RequestDistanceMsg__Sequence * array, size_t size);

/// Finalize array of msg/RequestDistanceMsg messages.
/**
 * It calls
 * custom_interface__msg__RequestDistanceMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
void
custom_interface__msg__RequestDistanceMsg__Sequence__fini(custom_interface__msg__RequestDistanceMsg__Sequence * array);

/// Create array of msg/RequestDistanceMsg messages.
/**
 * It allocates the memory for the array and calls
 * custom_interface__msg__RequestDistanceMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
custom_interface__msg__RequestDistanceMsg__Sequence *
custom_interface__msg__RequestDistanceMsg__Sequence__create(size_t size);

/// Destroy array of msg/RequestDistanceMsg messages.
/**
 * It calls
 * custom_interface__msg__RequestDistanceMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
void
custom_interface__msg__RequestDistanceMsg__Sequence__destroy(custom_interface__msg__RequestDistanceMsg__Sequence * array);

/// Check for msg/RequestDistanceMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
bool
custom_interface__msg__RequestDistanceMsg__Sequence__are_equal(const custom_interface__msg__RequestDistanceMsg__Sequence * lhs, const custom_interface__msg__RequestDistanceMsg__Sequence * rhs);

/// Copy an array of msg/RequestDistanceMsg messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_interface
bool
custom_interface__msg__RequestDistanceMsg__Sequence__copy(
  const custom_interface__msg__RequestDistanceMsg__Sequence * input,
  custom_interface__msg__RequestDistanceMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__REQUEST_DISTANCE_MSG__FUNCTIONS_H_
