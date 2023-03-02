// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zed_interfaces:msg/PlaneStamped.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/plane_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `mesh`
#include "shape_msgs/msg/detail/mesh__functions.h"
// Member `coefficients`
#include "shape_msgs/msg/detail/plane__functions.h"
// Member `normal`
// Member `center`
#include "geometry_msgs/msg/detail/point32__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/transform__functions.h"
// Member `bounds`
#include "geometry_msgs/msg/detail/polygon__functions.h"

bool
zed_interfaces__msg__PlaneStamped__init(zed_interfaces__msg__PlaneStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    zed_interfaces__msg__PlaneStamped__fini(msg);
    return false;
  }
  // mesh
  if (!shape_msgs__msg__Mesh__init(&msg->mesh)) {
    zed_interfaces__msg__PlaneStamped__fini(msg);
    return false;
  }
  // coefficients
  if (!shape_msgs__msg__Plane__init(&msg->coefficients)) {
    zed_interfaces__msg__PlaneStamped__fini(msg);
    return false;
  }
  // normal
  if (!geometry_msgs__msg__Point32__init(&msg->normal)) {
    zed_interfaces__msg__PlaneStamped__fini(msg);
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point32__init(&msg->center)) {
    zed_interfaces__msg__PlaneStamped__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Transform__init(&msg->pose)) {
    zed_interfaces__msg__PlaneStamped__fini(msg);
    return false;
  }
  // extents
  // bounds
  if (!geometry_msgs__msg__Polygon__init(&msg->bounds)) {
    zed_interfaces__msg__PlaneStamped__fini(msg);
    return false;
  }
  return true;
}

void
zed_interfaces__msg__PlaneStamped__fini(zed_interfaces__msg__PlaneStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // mesh
  shape_msgs__msg__Mesh__fini(&msg->mesh);
  // coefficients
  shape_msgs__msg__Plane__fini(&msg->coefficients);
  // normal
  geometry_msgs__msg__Point32__fini(&msg->normal);
  // center
  geometry_msgs__msg__Point32__fini(&msg->center);
  // pose
  geometry_msgs__msg__Transform__fini(&msg->pose);
  // extents
  // bounds
  geometry_msgs__msg__Polygon__fini(&msg->bounds);
}

bool
zed_interfaces__msg__PlaneStamped__are_equal(const zed_interfaces__msg__PlaneStamped * lhs, const zed_interfaces__msg__PlaneStamped * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // mesh
  if (!shape_msgs__msg__Mesh__are_equal(
      &(lhs->mesh), &(rhs->mesh)))
  {
    return false;
  }
  // coefficients
  if (!shape_msgs__msg__Plane__are_equal(
      &(lhs->coefficients), &(rhs->coefficients)))
  {
    return false;
  }
  // normal
  if (!geometry_msgs__msg__Point32__are_equal(
      &(lhs->normal), &(rhs->normal)))
  {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point32__are_equal(
      &(lhs->center), &(rhs->center)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Transform__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // extents
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->extents[i] != rhs->extents[i]) {
      return false;
    }
  }
  // bounds
  if (!geometry_msgs__msg__Polygon__are_equal(
      &(lhs->bounds), &(rhs->bounds)))
  {
    return false;
  }
  return true;
}

bool
zed_interfaces__msg__PlaneStamped__copy(
  const zed_interfaces__msg__PlaneStamped * input,
  zed_interfaces__msg__PlaneStamped * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // mesh
  if (!shape_msgs__msg__Mesh__copy(
      &(input->mesh), &(output->mesh)))
  {
    return false;
  }
  // coefficients
  if (!shape_msgs__msg__Plane__copy(
      &(input->coefficients), &(output->coefficients)))
  {
    return false;
  }
  // normal
  if (!geometry_msgs__msg__Point32__copy(
      &(input->normal), &(output->normal)))
  {
    return false;
  }
  // center
  if (!geometry_msgs__msg__Point32__copy(
      &(input->center), &(output->center)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Transform__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // extents
  for (size_t i = 0; i < 2; ++i) {
    output->extents[i] = input->extents[i];
  }
  // bounds
  if (!geometry_msgs__msg__Polygon__copy(
      &(input->bounds), &(output->bounds)))
  {
    return false;
  }
  return true;
}

zed_interfaces__msg__PlaneStamped *
zed_interfaces__msg__PlaneStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__PlaneStamped * msg = (zed_interfaces__msg__PlaneStamped *)allocator.allocate(sizeof(zed_interfaces__msg__PlaneStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zed_interfaces__msg__PlaneStamped));
  bool success = zed_interfaces__msg__PlaneStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zed_interfaces__msg__PlaneStamped__destroy(zed_interfaces__msg__PlaneStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zed_interfaces__msg__PlaneStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zed_interfaces__msg__PlaneStamped__Sequence__init(zed_interfaces__msg__PlaneStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__PlaneStamped * data = NULL;

  if (size) {
    data = (zed_interfaces__msg__PlaneStamped *)allocator.zero_allocate(size, sizeof(zed_interfaces__msg__PlaneStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zed_interfaces__msg__PlaneStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zed_interfaces__msg__PlaneStamped__fini(&data[i - 1]);
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
zed_interfaces__msg__PlaneStamped__Sequence__fini(zed_interfaces__msg__PlaneStamped__Sequence * array)
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
      zed_interfaces__msg__PlaneStamped__fini(&array->data[i]);
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

zed_interfaces__msg__PlaneStamped__Sequence *
zed_interfaces__msg__PlaneStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__PlaneStamped__Sequence * array = (zed_interfaces__msg__PlaneStamped__Sequence *)allocator.allocate(sizeof(zed_interfaces__msg__PlaneStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zed_interfaces__msg__PlaneStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zed_interfaces__msg__PlaneStamped__Sequence__destroy(zed_interfaces__msg__PlaneStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zed_interfaces__msg__PlaneStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zed_interfaces__msg__PlaneStamped__Sequence__are_equal(const zed_interfaces__msg__PlaneStamped__Sequence * lhs, const zed_interfaces__msg__PlaneStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zed_interfaces__msg__PlaneStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zed_interfaces__msg__PlaneStamped__Sequence__copy(
  const zed_interfaces__msg__PlaneStamped__Sequence * input,
  zed_interfaces__msg__PlaneStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zed_interfaces__msg__PlaneStamped);
    zed_interfaces__msg__PlaneStamped * data =
      (zed_interfaces__msg__PlaneStamped *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zed_interfaces__msg__PlaneStamped__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          zed_interfaces__msg__PlaneStamped__fini(&data[i]);
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
    if (!zed_interfaces__msg__PlaneStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
