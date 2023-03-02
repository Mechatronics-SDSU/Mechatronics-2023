// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zed_interfaces:msg/Skeleton2D.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/skeleton2_d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `keypoints`
#include "zed_interfaces/msg/detail/keypoint2_df__functions.h"

bool
zed_interfaces__msg__Skeleton2D__init(zed_interfaces__msg__Skeleton2D * msg)
{
  if (!msg) {
    return false;
  }
  // keypoints
  for (size_t i = 0; i < 34; ++i) {
    if (!zed_interfaces__msg__Keypoint2Df__init(&msg->keypoints[i])) {
      zed_interfaces__msg__Skeleton2D__fini(msg);
      return false;
    }
  }
  return true;
}

void
zed_interfaces__msg__Skeleton2D__fini(zed_interfaces__msg__Skeleton2D * msg)
{
  if (!msg) {
    return;
  }
  // keypoints
  for (size_t i = 0; i < 34; ++i) {
    zed_interfaces__msg__Keypoint2Df__fini(&msg->keypoints[i]);
  }
}

bool
zed_interfaces__msg__Skeleton2D__are_equal(const zed_interfaces__msg__Skeleton2D * lhs, const zed_interfaces__msg__Skeleton2D * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // keypoints
  for (size_t i = 0; i < 34; ++i) {
    if (!zed_interfaces__msg__Keypoint2Df__are_equal(
        &(lhs->keypoints[i]), &(rhs->keypoints[i])))
    {
      return false;
    }
  }
  return true;
}

bool
zed_interfaces__msg__Skeleton2D__copy(
  const zed_interfaces__msg__Skeleton2D * input,
  zed_interfaces__msg__Skeleton2D * output)
{
  if (!input || !output) {
    return false;
  }
  // keypoints
  for (size_t i = 0; i < 34; ++i) {
    if (!zed_interfaces__msg__Keypoint2Df__copy(
        &(input->keypoints[i]), &(output->keypoints[i])))
    {
      return false;
    }
  }
  return true;
}

zed_interfaces__msg__Skeleton2D *
zed_interfaces__msg__Skeleton2D__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__Skeleton2D * msg = (zed_interfaces__msg__Skeleton2D *)allocator.allocate(sizeof(zed_interfaces__msg__Skeleton2D), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zed_interfaces__msg__Skeleton2D));
  bool success = zed_interfaces__msg__Skeleton2D__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zed_interfaces__msg__Skeleton2D__destroy(zed_interfaces__msg__Skeleton2D * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zed_interfaces__msg__Skeleton2D__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zed_interfaces__msg__Skeleton2D__Sequence__init(zed_interfaces__msg__Skeleton2D__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__Skeleton2D * data = NULL;

  if (size) {
    data = (zed_interfaces__msg__Skeleton2D *)allocator.zero_allocate(size, sizeof(zed_interfaces__msg__Skeleton2D), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zed_interfaces__msg__Skeleton2D__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zed_interfaces__msg__Skeleton2D__fini(&data[i - 1]);
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
zed_interfaces__msg__Skeleton2D__Sequence__fini(zed_interfaces__msg__Skeleton2D__Sequence * array)
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
      zed_interfaces__msg__Skeleton2D__fini(&array->data[i]);
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

zed_interfaces__msg__Skeleton2D__Sequence *
zed_interfaces__msg__Skeleton2D__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__Skeleton2D__Sequence * array = (zed_interfaces__msg__Skeleton2D__Sequence *)allocator.allocate(sizeof(zed_interfaces__msg__Skeleton2D__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zed_interfaces__msg__Skeleton2D__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zed_interfaces__msg__Skeleton2D__Sequence__destroy(zed_interfaces__msg__Skeleton2D__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zed_interfaces__msg__Skeleton2D__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zed_interfaces__msg__Skeleton2D__Sequence__are_equal(const zed_interfaces__msg__Skeleton2D__Sequence * lhs, const zed_interfaces__msg__Skeleton2D__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zed_interfaces__msg__Skeleton2D__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zed_interfaces__msg__Skeleton2D__Sequence__copy(
  const zed_interfaces__msg__Skeleton2D__Sequence * input,
  zed_interfaces__msg__Skeleton2D__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zed_interfaces__msg__Skeleton2D);
    zed_interfaces__msg__Skeleton2D * data =
      (zed_interfaces__msg__Skeleton2D *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zed_interfaces__msg__Skeleton2D__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          zed_interfaces__msg__Skeleton2D__fini(&data[i]);
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
    if (!zed_interfaces__msg__Skeleton2D__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
