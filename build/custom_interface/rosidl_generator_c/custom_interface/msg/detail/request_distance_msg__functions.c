// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interface:msg/RequestDistanceMsg.idl
// generated code does not contain a copyright notice
#include "custom_interface/msg/detail/request_distance_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `start_point`
// Member `end_point`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
custom_interface__msg__RequestDistanceMsg__init(custom_interface__msg__RequestDistanceMsg * msg)
{
  if (!msg) {
    return false;
  }
  // start_point
  if (!geometry_msgs__msg__Point__init(&msg->start_point)) {
    custom_interface__msg__RequestDistanceMsg__fini(msg);
    return false;
  }
  // end_point
  if (!geometry_msgs__msg__Point__init(&msg->end_point)) {
    custom_interface__msg__RequestDistanceMsg__fini(msg);
    return false;
  }
  return true;
}

void
custom_interface__msg__RequestDistanceMsg__fini(custom_interface__msg__RequestDistanceMsg * msg)
{
  if (!msg) {
    return;
  }
  // start_point
  geometry_msgs__msg__Point__fini(&msg->start_point);
  // end_point
  geometry_msgs__msg__Point__fini(&msg->end_point);
}

bool
custom_interface__msg__RequestDistanceMsg__are_equal(const custom_interface__msg__RequestDistanceMsg * lhs, const custom_interface__msg__RequestDistanceMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start_point
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->start_point), &(rhs->start_point)))
  {
    return false;
  }
  // end_point
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->end_point), &(rhs->end_point)))
  {
    return false;
  }
  return true;
}

bool
custom_interface__msg__RequestDistanceMsg__copy(
  const custom_interface__msg__RequestDistanceMsg * input,
  custom_interface__msg__RequestDistanceMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // start_point
  if (!geometry_msgs__msg__Point__copy(
      &(input->start_point), &(output->start_point)))
  {
    return false;
  }
  // end_point
  if (!geometry_msgs__msg__Point__copy(
      &(input->end_point), &(output->end_point)))
  {
    return false;
  }
  return true;
}

custom_interface__msg__RequestDistanceMsg *
custom_interface__msg__RequestDistanceMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__RequestDistanceMsg * msg = (custom_interface__msg__RequestDistanceMsg *)allocator.allocate(sizeof(custom_interface__msg__RequestDistanceMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interface__msg__RequestDistanceMsg));
  bool success = custom_interface__msg__RequestDistanceMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interface__msg__RequestDistanceMsg__destroy(custom_interface__msg__RequestDistanceMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interface__msg__RequestDistanceMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interface__msg__RequestDistanceMsg__Sequence__init(custom_interface__msg__RequestDistanceMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__RequestDistanceMsg * data = NULL;

  if (size) {
    data = (custom_interface__msg__RequestDistanceMsg *)allocator.zero_allocate(size, sizeof(custom_interface__msg__RequestDistanceMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interface__msg__RequestDistanceMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interface__msg__RequestDistanceMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_interface__msg__RequestDistanceMsg__Sequence__fini(custom_interface__msg__RequestDistanceMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_interface__msg__RequestDistanceMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_interface__msg__RequestDistanceMsg__Sequence *
custom_interface__msg__RequestDistanceMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__RequestDistanceMsg__Sequence * array = (custom_interface__msg__RequestDistanceMsg__Sequence *)allocator.allocate(sizeof(custom_interface__msg__RequestDistanceMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interface__msg__RequestDistanceMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interface__msg__RequestDistanceMsg__Sequence__destroy(custom_interface__msg__RequestDistanceMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interface__msg__RequestDistanceMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interface__msg__RequestDistanceMsg__Sequence__are_equal(const custom_interface__msg__RequestDistanceMsg__Sequence * lhs, const custom_interface__msg__RequestDistanceMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interface__msg__RequestDistanceMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interface__msg__RequestDistanceMsg__Sequence__copy(
  const custom_interface__msg__RequestDistanceMsg__Sequence * input,
  custom_interface__msg__RequestDistanceMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interface__msg__RequestDistanceMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interface__msg__RequestDistanceMsg * data =
      (custom_interface__msg__RequestDistanceMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interface__msg__RequestDistanceMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interface__msg__RequestDistanceMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interface__msg__RequestDistanceMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
