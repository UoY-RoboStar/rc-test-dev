// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sample_interface:msg/Output.idl
// generated code does not contain a copyright notice
#include "sample_interface/msg/detail/output__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
sample_interface__msg__Output__init(sample_interface__msg__Output * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    sample_interface__msg__Output__fini(msg);
    return false;
  }
  // value
  return true;
}

void
sample_interface__msg__Output__fini(sample_interface__msg__Output * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // value
}

bool
sample_interface__msg__Output__are_equal(const sample_interface__msg__Output * lhs, const sample_interface__msg__Output * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // value
  if (lhs->value != rhs->value) {
    return false;
  }
  return true;
}

bool
sample_interface__msg__Output__copy(
  const sample_interface__msg__Output * input,
  sample_interface__msg__Output * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // value
  output->value = input->value;
  return true;
}

sample_interface__msg__Output *
sample_interface__msg__Output__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interface__msg__Output * msg = (sample_interface__msg__Output *)allocator.allocate(sizeof(sample_interface__msg__Output), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sample_interface__msg__Output));
  bool success = sample_interface__msg__Output__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sample_interface__msg__Output__destroy(sample_interface__msg__Output * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sample_interface__msg__Output__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sample_interface__msg__Output__Sequence__init(sample_interface__msg__Output__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interface__msg__Output * data = NULL;

  if (size) {
    data = (sample_interface__msg__Output *)allocator.zero_allocate(size, sizeof(sample_interface__msg__Output), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sample_interface__msg__Output__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sample_interface__msg__Output__fini(&data[i - 1]);
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
sample_interface__msg__Output__Sequence__fini(sample_interface__msg__Output__Sequence * array)
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
      sample_interface__msg__Output__fini(&array->data[i]);
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

sample_interface__msg__Output__Sequence *
sample_interface__msg__Output__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sample_interface__msg__Output__Sequence * array = (sample_interface__msg__Output__Sequence *)allocator.allocate(sizeof(sample_interface__msg__Output__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sample_interface__msg__Output__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sample_interface__msg__Output__Sequence__destroy(sample_interface__msg__Output__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sample_interface__msg__Output__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sample_interface__msg__Output__Sequence__are_equal(const sample_interface__msg__Output__Sequence * lhs, const sample_interface__msg__Output__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sample_interface__msg__Output__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sample_interface__msg__Output__Sequence__copy(
  const sample_interface__msg__Output__Sequence * input,
  sample_interface__msg__Output__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sample_interface__msg__Output);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sample_interface__msg__Output * data =
      (sample_interface__msg__Output *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sample_interface__msg__Output__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sample_interface__msg__Output__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sample_interface__msg__Output__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
