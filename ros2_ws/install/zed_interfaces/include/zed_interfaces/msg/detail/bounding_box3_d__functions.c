// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zed_interfaces:msg/BoundingBox3D.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/bounding_box3_d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `corners`
#include "zed_interfaces/msg/detail/keypoint3_d__functions.h"

bool
zed_interfaces__msg__BoundingBox3D__init(zed_interfaces__msg__BoundingBox3D * msg)
{
  if (!msg) {
    return false;
  }
  // corners
  for (size_t i = 0; i < 8; ++i) {
    if (!zed_interfaces__msg__Keypoint3D__init(&msg->corners[i])) {
      zed_interfaces__msg__BoundingBox3D__fini(msg);
      return false;
    }
  }
  return true;
}

void
zed_interfaces__msg__BoundingBox3D__fini(zed_interfaces__msg__BoundingBox3D * msg)
{
  if (!msg) {
    return;
  }
  // corners
  for (size_t i = 0; i < 8; ++i) {
    zed_interfaces__msg__Keypoint3D__fini(&msg->corners[i]);
  }
}

bool
zed_interfaces__msg__BoundingBox3D__are_equal(const zed_interfaces__msg__BoundingBox3D * lhs, const zed_interfaces__msg__BoundingBox3D * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // corners
  for (size_t i = 0; i < 8; ++i) {
    if (!zed_interfaces__msg__Keypoint3D__are_equal(
        &(lhs->corners[i]), &(rhs->corners[i])))
    {
      return false;
    }
  }
  return true;
}

bool
zed_interfaces__msg__BoundingBox3D__copy(
  const zed_interfaces__msg__BoundingBox3D * input,
  zed_interfaces__msg__BoundingBox3D * output)
{
  if (!input || !output) {
    return false;
  }
  // corners
  for (size_t i = 0; i < 8; ++i) {
    if (!zed_interfaces__msg__Keypoint3D__copy(
        &(input->corners[i]), &(output->corners[i])))
    {
      return false;
    }
  }
  return true;
}

zed_interfaces__msg__BoundingBox3D *
zed_interfaces__msg__BoundingBox3D__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__BoundingBox3D * msg = (zed_interfaces__msg__BoundingBox3D *)allocator.allocate(sizeof(zed_interfaces__msg__BoundingBox3D), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zed_interfaces__msg__BoundingBox3D));
  bool success = zed_interfaces__msg__BoundingBox3D__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zed_interfaces__msg__BoundingBox3D__destroy(zed_interfaces__msg__BoundingBox3D * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zed_interfaces__msg__BoundingBox3D__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zed_interfaces__msg__BoundingBox3D__Sequence__init(zed_interfaces__msg__BoundingBox3D__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__BoundingBox3D * data = NULL;

  if (size) {
    data = (zed_interfaces__msg__BoundingBox3D *)allocator.zero_allocate(size, sizeof(zed_interfaces__msg__BoundingBox3D), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zed_interfaces__msg__BoundingBox3D__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zed_interfaces__msg__BoundingBox3D__fini(&data[i - 1]);
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
zed_interfaces__msg__BoundingBox3D__Sequence__fini(zed_interfaces__msg__BoundingBox3D__Sequence * array)
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
      zed_interfaces__msg__BoundingBox3D__fini(&array->data[i]);
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

zed_interfaces__msg__BoundingBox3D__Sequence *
zed_interfaces__msg__BoundingBox3D__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__BoundingBox3D__Sequence * array = (zed_interfaces__msg__BoundingBox3D__Sequence *)allocator.allocate(sizeof(zed_interfaces__msg__BoundingBox3D__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zed_interfaces__msg__BoundingBox3D__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zed_interfaces__msg__BoundingBox3D__Sequence__destroy(zed_interfaces__msg__BoundingBox3D__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zed_interfaces__msg__BoundingBox3D__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zed_interfaces__msg__BoundingBox3D__Sequence__are_equal(const zed_interfaces__msg__BoundingBox3D__Sequence * lhs, const zed_interfaces__msg__BoundingBox3D__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zed_interfaces__msg__BoundingBox3D__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zed_interfaces__msg__BoundingBox3D__Sequence__copy(
  const zed_interfaces__msg__BoundingBox3D__Sequence * input,
  zed_interfaces__msg__BoundingBox3D__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zed_interfaces__msg__BoundingBox3D);
    zed_interfaces__msg__BoundingBox3D * data =
      (zed_interfaces__msg__BoundingBox3D *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zed_interfaces__msg__BoundingBox3D__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          zed_interfaces__msg__BoundingBox3D__fini(&data[i]);
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
    if (!zed_interfaces__msg__BoundingBox3D__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
