// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interface:msg/ResponseDistanceMsg.idl
// generated code does not contain a copyright notice
#include "custom_interface/msg/detail/response_distance_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `place_holder`
#include "rosidl_runtime_c/string_functions.h"

bool
custom_interface__msg__ResponseDistanceMsg__init(custom_interface__msg__ResponseDistanceMsg * msg)
{
  if (!msg) {
    return false;
  }
  // place_holder
  if (!rosidl_runtime_c__String__init(&msg->place_holder)) {
    custom_interface__msg__ResponseDistanceMsg__fini(msg);
    return false;
  }
  return true;
}

void
custom_interface__msg__ResponseDistanceMsg__fini(custom_interface__msg__ResponseDistanceMsg * msg)
{
  if (!msg) {
    return;
  }
  // place_holder
  rosidl_runtime_c__String__fini(&msg->place_holder);
}

bool
custom_interface__msg__ResponseDistanceMsg__are_equal(const custom_interface__msg__ResponseDistanceMsg * lhs, const custom_interface__msg__ResponseDistanceMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // place_holder
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->place_holder), &(rhs->place_holder)))
  {
    return false;
  }
  return true;
}

bool
custom_interface__msg__ResponseDistanceMsg__copy(
  const custom_interface__msg__ResponseDistanceMsg * input,
  custom_interface__msg__ResponseDistanceMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // place_holder
  if (!rosidl_runtime_c__String__copy(
      &(input->place_holder), &(output->place_holder)))
  {
    return false;
  }
  return true;
}

custom_interface__msg__ResponseDistanceMsg *
custom_interface__msg__ResponseDistanceMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__ResponseDistanceMsg * msg = (custom_interface__msg__ResponseDistanceMsg *)allocator.allocate(sizeof(custom_interface__msg__ResponseDistanceMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interface__msg__ResponseDistanceMsg));
  bool success = custom_interface__msg__ResponseDistanceMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interface__msg__ResponseDistanceMsg__destroy(custom_interface__msg__ResponseDistanceMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interface__msg__ResponseDistanceMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interface__msg__ResponseDistanceMsg__Sequence__init(custom_interface__msg__ResponseDistanceMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__ResponseDistanceMsg * data = NULL;

  if (size) {
    data = (custom_interface__msg__ResponseDistanceMsg *)allocator.zero_allocate(size, sizeof(custom_interface__msg__ResponseDistanceMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interface__msg__ResponseDistanceMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interface__msg__ResponseDistanceMsg__fini(&data[i - 1]);
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
custom_interface__msg__ResponseDistanceMsg__Sequence__fini(custom_interface__msg__ResponseDistanceMsg__Sequence * array)
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
      custom_interface__msg__ResponseDistanceMsg__fini(&array->data[i]);
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

custom_interface__msg__ResponseDistanceMsg__Sequence *
custom_interface__msg__ResponseDistanceMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__ResponseDistanceMsg__Sequence * array = (custom_interface__msg__ResponseDistanceMsg__Sequence *)allocator.allocate(sizeof(custom_interface__msg__ResponseDistanceMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interface__msg__ResponseDistanceMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interface__msg__ResponseDistanceMsg__Sequence__destroy(custom_interface__msg__ResponseDistanceMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interface__msg__ResponseDistanceMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interface__msg__ResponseDistanceMsg__Sequence__are_equal(const custom_interface__msg__ResponseDistanceMsg__Sequence * lhs, const custom_interface__msg__ResponseDistanceMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interface__msg__ResponseDistanceMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interface__msg__ResponseDistanceMsg__Sequence__copy(
  const custom_interface__msg__ResponseDistanceMsg__Sequence * input,
  custom_interface__msg__ResponseDistanceMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interface__msg__ResponseDistanceMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interface__msg__ResponseDistanceMsg * data =
      (custom_interface__msg__ResponseDistanceMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interface__msg__ResponseDistanceMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interface__msg__ResponseDistanceMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interface__msg__ResponseDistanceMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
