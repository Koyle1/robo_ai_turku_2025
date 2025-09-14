// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interface:msg/Count.idl
// generated code does not contain a copyright notice
#include "custom_interface/msg/detail/count__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_interface__msg__Count__init(custom_interface__msg__Count * msg)
{
  if (!msg) {
    return false;
  }
  // end_count
  // current_count
  return true;
}

void
custom_interface__msg__Count__fini(custom_interface__msg__Count * msg)
{
  if (!msg) {
    return;
  }
  // end_count
  // current_count
}

bool
custom_interface__msg__Count__are_equal(const custom_interface__msg__Count * lhs, const custom_interface__msg__Count * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // end_count
  if (lhs->end_count != rhs->end_count) {
    return false;
  }
  // current_count
  if (lhs->current_count != rhs->current_count) {
    return false;
  }
  return true;
}

bool
custom_interface__msg__Count__copy(
  const custom_interface__msg__Count * input,
  custom_interface__msg__Count * output)
{
  if (!input || !output) {
    return false;
  }
  // end_count
  output->end_count = input->end_count;
  // current_count
  output->current_count = input->current_count;
  return true;
}

custom_interface__msg__Count *
custom_interface__msg__Count__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__Count * msg = (custom_interface__msg__Count *)allocator.allocate(sizeof(custom_interface__msg__Count), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interface__msg__Count));
  bool success = custom_interface__msg__Count__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interface__msg__Count__destroy(custom_interface__msg__Count * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interface__msg__Count__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interface__msg__Count__Sequence__init(custom_interface__msg__Count__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__Count * data = NULL;

  if (size) {
    data = (custom_interface__msg__Count *)allocator.zero_allocate(size, sizeof(custom_interface__msg__Count), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interface__msg__Count__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interface__msg__Count__fini(&data[i - 1]);
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
custom_interface__msg__Count__Sequence__fini(custom_interface__msg__Count__Sequence * array)
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
      custom_interface__msg__Count__fini(&array->data[i]);
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

custom_interface__msg__Count__Sequence *
custom_interface__msg__Count__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__Count__Sequence * array = (custom_interface__msg__Count__Sequence *)allocator.allocate(sizeof(custom_interface__msg__Count__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interface__msg__Count__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interface__msg__Count__Sequence__destroy(custom_interface__msg__Count__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interface__msg__Count__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interface__msg__Count__Sequence__are_equal(const custom_interface__msg__Count__Sequence * lhs, const custom_interface__msg__Count__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interface__msg__Count__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interface__msg__Count__Sequence__copy(
  const custom_interface__msg__Count__Sequence * input,
  custom_interface__msg__Count__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interface__msg__Count);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interface__msg__Count * data =
      (custom_interface__msg__Count *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interface__msg__Count__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interface__msg__Count__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interface__msg__Count__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
