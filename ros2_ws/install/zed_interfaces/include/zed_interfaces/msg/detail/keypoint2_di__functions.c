// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zed_interfaces:msg/Keypoint2Di.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/keypoint2_di__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
zed_interfaces__msg__Keypoint2Di__init(zed_interfaces__msg__Keypoint2Di * msg)
{
  if (!msg) {
    return false;
  }
  // kp
  return true;
}

void
zed_interfaces__msg__Keypoint2Di__fini(zed_interfaces__msg__Keypoint2Di * msg)
{
  if (!msg) {
    return;
  }
  // kp
}

bool
zed_interfaces__msg__Keypoint2Di__are_equal(const zed_interfaces__msg__Keypoint2Di * lhs, const zed_interfaces__msg__Keypoint2Di * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // kp
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->kp[i] != rhs->kp[i]) {
      return false;
    }
  }
  return true;
}

bool
zed_interfaces__msg__Keypoint2Di__copy(
  const zed_interfaces__msg__Keypoint2Di * input,
  zed_interfaces__msg__Keypoint2Di * output)
{
  if (!input || !output) {
    return false;
  }
  // kp
  for (size_t i = 0; i < 2; ++i) {
    output->kp[i] = input->kp[i];
  }
  return true;
}

zed_interfaces__msg__Keypoint2Di *
zed_interfaces__msg__Keypoint2Di__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__Keypoint2Di * msg = (zed_interfaces__msg__Keypoint2Di *)allocator.allocate(sizeof(zed_interfaces__msg__Keypoint2Di), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zed_interfaces__msg__Keypoint2Di));
  bool success = zed_interfaces__msg__Keypoint2Di__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zed_interfaces__msg__Keypoint2Di__destroy(zed_interfaces__msg__Keypoint2Di * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zed_interfaces__msg__Keypoint2Di__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zed_interfaces__msg__Keypoint2Di__Sequence__init(zed_interfaces__msg__Keypoint2Di__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__Keypoint2Di * data = NULL;

  if (size) {
    data = (zed_interfaces__msg__Keypoint2Di *)allocator.zero_allocate(size, sizeof(zed_interfaces__msg__Keypoint2Di), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zed_interfaces__msg__Keypoint2Di__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zed_interfaces__msg__Keypoint2Di__fini(&data[i - 1]);
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
zed_interfaces__msg__Keypoint2Di__Sequence__fini(zed_interfaces__msg__Keypoint2Di__Sequence * array)
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
      zed_interfaces__msg__Keypoint2Di__fini(&array->data[i]);
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

zed_interfaces__msg__Keypoint2Di__Sequence *
zed_interfaces__msg__Keypoint2Di__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__Keypoint2Di__Sequence * array = (zed_interfaces__msg__Keypoint2Di__Sequence *)allocator.allocate(sizeof(zed_interfaces__msg__Keypoint2Di__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zed_interfaces__msg__Keypoint2Di__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zed_interfaces__msg__Keypoint2Di__Sequence__destroy(zed_interfaces__msg__Keypoint2Di__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zed_interfaces__msg__Keypoint2Di__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zed_interfaces__msg__Keypoint2Di__Sequence__are_equal(const zed_interfaces__msg__Keypoint2Di__Sequence * lhs, const zed_interfaces__msg__Keypoint2Di__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zed_interfaces__msg__Keypoint2Di__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zed_interfaces__msg__Keypoint2Di__Sequence__copy(
  const zed_interfaces__msg__Keypoint2Di__Sequence * input,
  zed_interfaces__msg__Keypoint2Di__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zed_interfaces__msg__Keypoint2Di);
    zed_interfaces__msg__Keypoint2Di * data =
      (zed_interfaces__msg__Keypoint2Di *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zed_interfaces__msg__Keypoint2Di__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          zed_interfaces__msg__Keypoint2Di__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zed_interfaces__msg__Keypoint2Di__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
