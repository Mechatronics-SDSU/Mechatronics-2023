// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zed_interfaces:msg/Object.idl
// generated code does not contain a copyright notice
#include "zed_interfaces/msg/detail/object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `label`
// Member `sublabel`
#include "rosidl_runtime_c/string_functions.h"
// Member `bounding_box_2d`
#include "zed_interfaces/msg/detail/bounding_box2_di__functions.h"
// Member `bounding_box_3d`
// Member `head_bounding_box_3d`
#include "zed_interfaces/msg/detail/bounding_box3_d__functions.h"
// Member `head_bounding_box_2d`
#include "zed_interfaces/msg/detail/bounding_box2_df__functions.h"
// Member `skeleton_2d`
#include "zed_interfaces/msg/detail/skeleton2_d__functions.h"
// Member `skeleton_3d`
#include "zed_interfaces/msg/detail/skeleton3_d__functions.h"

bool
zed_interfaces__msg__Object__init(zed_interfaces__msg__Object * msg)
{
  if (!msg) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    zed_interfaces__msg__Object__fini(msg);
    return false;
  }
  // label_id
  // sublabel
  if (!rosidl_runtime_c__String__init(&msg->sublabel)) {
    zed_interfaces__msg__Object__fini(msg);
    return false;
  }
  // confidence
  // position
  // position_covariance
  // velocity
  // tracking_available
  // tracking_state
  // action_state
  // bounding_box_2d
  if (!zed_interfaces__msg__BoundingBox2Di__init(&msg->bounding_box_2d)) {
    zed_interfaces__msg__Object__fini(msg);
    return false;
  }
  // bounding_box_3d
  if (!zed_interfaces__msg__BoundingBox3D__init(&msg->bounding_box_3d)) {
    zed_interfaces__msg__Object__fini(msg);
    return false;
  }
  // dimensions_3d
  // skeleton_available
  // body_format
  // head_bounding_box_2d
  if (!zed_interfaces__msg__BoundingBox2Df__init(&msg->head_bounding_box_2d)) {
    zed_interfaces__msg__Object__fini(msg);
    return false;
  }
  // head_bounding_box_3d
  if (!zed_interfaces__msg__BoundingBox3D__init(&msg->head_bounding_box_3d)) {
    zed_interfaces__msg__Object__fini(msg);
    return false;
  }
  // head_position
  // skeleton_2d
  if (!zed_interfaces__msg__Skeleton2D__init(&msg->skeleton_2d)) {
    zed_interfaces__msg__Object__fini(msg);
    return false;
  }
  // skeleton_3d
  if (!zed_interfaces__msg__Skeleton3D__init(&msg->skeleton_3d)) {
    zed_interfaces__msg__Object__fini(msg);
    return false;
  }
  return true;
}

void
zed_interfaces__msg__Object__fini(zed_interfaces__msg__Object * msg)
{
  if (!msg) {
    return;
  }
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // label_id
  // sublabel
  rosidl_runtime_c__String__fini(&msg->sublabel);
  // confidence
  // position
  // position_covariance
  // velocity
  // tracking_available
  // tracking_state
  // action_state
  // bounding_box_2d
  zed_interfaces__msg__BoundingBox2Di__fini(&msg->bounding_box_2d);
  // bounding_box_3d
  zed_interfaces__msg__BoundingBox3D__fini(&msg->bounding_box_3d);
  // dimensions_3d
  // skeleton_available
  // body_format
  // head_bounding_box_2d
  zed_interfaces__msg__BoundingBox2Df__fini(&msg->head_bounding_box_2d);
  // head_bounding_box_3d
  zed_interfaces__msg__BoundingBox3D__fini(&msg->head_bounding_box_3d);
  // head_position
  // skeleton_2d
  zed_interfaces__msg__Skeleton2D__fini(&msg->skeleton_2d);
  // skeleton_3d
  zed_interfaces__msg__Skeleton3D__fini(&msg->skeleton_3d);
}

bool
zed_interfaces__msg__Object__are_equal(const zed_interfaces__msg__Object * lhs, const zed_interfaces__msg__Object * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // label_id
  if (lhs->label_id != rhs->label_id) {
    return false;
  }
  // sublabel
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sublabel), &(rhs->sublabel)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // position
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->position[i] != rhs->position[i]) {
      return false;
    }
  }
  // position_covariance
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->position_covariance[i] != rhs->position_covariance[i]) {
      return false;
    }
  }
  // velocity
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->velocity[i] != rhs->velocity[i]) {
      return false;
    }
  }
  // tracking_available
  if (lhs->tracking_available != rhs->tracking_available) {
    return false;
  }
  // tracking_state
  if (lhs->tracking_state != rhs->tracking_state) {
    return false;
  }
  // action_state
  if (lhs->action_state != rhs->action_state) {
    return false;
  }
  // bounding_box_2d
  if (!zed_interfaces__msg__BoundingBox2Di__are_equal(
      &(lhs->bounding_box_2d), &(rhs->bounding_box_2d)))
  {
    return false;
  }
  // bounding_box_3d
  if (!zed_interfaces__msg__BoundingBox3D__are_equal(
      &(lhs->bounding_box_3d), &(rhs->bounding_box_3d)))
  {
    return false;
  }
  // dimensions_3d
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->dimensions_3d[i] != rhs->dimensions_3d[i]) {
      return false;
    }
  }
  // skeleton_available
  if (lhs->skeleton_available != rhs->skeleton_available) {
    return false;
  }
  // body_format
  if (lhs->body_format != rhs->body_format) {
    return false;
  }
  // head_bounding_box_2d
  if (!zed_interfaces__msg__BoundingBox2Df__are_equal(
      &(lhs->head_bounding_box_2d), &(rhs->head_bounding_box_2d)))
  {
    return false;
  }
  // head_bounding_box_3d
  if (!zed_interfaces__msg__BoundingBox3D__are_equal(
      &(lhs->head_bounding_box_3d), &(rhs->head_bounding_box_3d)))
  {
    return false;
  }
  // head_position
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->head_position[i] != rhs->head_position[i]) {
      return false;
    }
  }
  // skeleton_2d
  if (!zed_interfaces__msg__Skeleton2D__are_equal(
      &(lhs->skeleton_2d), &(rhs->skeleton_2d)))
  {
    return false;
  }
  // skeleton_3d
  if (!zed_interfaces__msg__Skeleton3D__are_equal(
      &(lhs->skeleton_3d), &(rhs->skeleton_3d)))
  {
    return false;
  }
  return true;
}

bool
zed_interfaces__msg__Object__copy(
  const zed_interfaces__msg__Object * input,
  zed_interfaces__msg__Object * output)
{
  if (!input || !output) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // label_id
  output->label_id = input->label_id;
  // sublabel
  if (!rosidl_runtime_c__String__copy(
      &(input->sublabel), &(output->sublabel)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // position
  for (size_t i = 0; i < 3; ++i) {
    output->position[i] = input->position[i];
  }
  // position_covariance
  for (size_t i = 0; i < 6; ++i) {
    output->position_covariance[i] = input->position_covariance[i];
  }
  // velocity
  for (size_t i = 0; i < 3; ++i) {
    output->velocity[i] = input->velocity[i];
  }
  // tracking_available
  output->tracking_available = input->tracking_available;
  // tracking_state
  output->tracking_state = input->tracking_state;
  // action_state
  output->action_state = input->action_state;
  // bounding_box_2d
  if (!zed_interfaces__msg__BoundingBox2Di__copy(
      &(input->bounding_box_2d), &(output->bounding_box_2d)))
  {
    return false;
  }
  // bounding_box_3d
  if (!zed_interfaces__msg__BoundingBox3D__copy(
      &(input->bounding_box_3d), &(output->bounding_box_3d)))
  {
    return false;
  }
  // dimensions_3d
  for (size_t i = 0; i < 3; ++i) {
    output->dimensions_3d[i] = input->dimensions_3d[i];
  }
  // skeleton_available
  output->skeleton_available = input->skeleton_available;
  // body_format
  output->body_format = input->body_format;
  // head_bounding_box_2d
  if (!zed_interfaces__msg__BoundingBox2Df__copy(
      &(input->head_bounding_box_2d), &(output->head_bounding_box_2d)))
  {
    return false;
  }
  // head_bounding_box_3d
  if (!zed_interfaces__msg__BoundingBox3D__copy(
      &(input->head_bounding_box_3d), &(output->head_bounding_box_3d)))
  {
    return false;
  }
  // head_position
  for (size_t i = 0; i < 3; ++i) {
    output->head_position[i] = input->head_position[i];
  }
  // skeleton_2d
  if (!zed_interfaces__msg__Skeleton2D__copy(
      &(input->skeleton_2d), &(output->skeleton_2d)))
  {
    return false;
  }
  // skeleton_3d
  if (!zed_interfaces__msg__Skeleton3D__copy(
      &(input->skeleton_3d), &(output->skeleton_3d)))
  {
    return false;
  }
  return true;
}

zed_interfaces__msg__Object *
zed_interfaces__msg__Object__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__Object * msg = (zed_interfaces__msg__Object *)allocator.allocate(sizeof(zed_interfaces__msg__Object), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zed_interfaces__msg__Object));
  bool success = zed_interfaces__msg__Object__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zed_interfaces__msg__Object__destroy(zed_interfaces__msg__Object * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zed_interfaces__msg__Object__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zed_interfaces__msg__Object__Sequence__init(zed_interfaces__msg__Object__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__Object * data = NULL;

  if (size) {
    data = (zed_interfaces__msg__Object *)allocator.zero_allocate(size, sizeof(zed_interfaces__msg__Object), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zed_interfaces__msg__Object__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zed_interfaces__msg__Object__fini(&data[i - 1]);
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
zed_interfaces__msg__Object__Sequence__fini(zed_interfaces__msg__Object__Sequence * array)
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
      zed_interfaces__msg__Object__fini(&array->data[i]);
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

zed_interfaces__msg__Object__Sequence *
zed_interfaces__msg__Object__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zed_interfaces__msg__Object__Sequence * array = (zed_interfaces__msg__Object__Sequence *)allocator.allocate(sizeof(zed_interfaces__msg__Object__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zed_interfaces__msg__Object__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zed_interfaces__msg__Object__Sequence__destroy(zed_interfaces__msg__Object__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zed_interfaces__msg__Object__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zed_interfaces__msg__Object__Sequence__are_equal(const zed_interfaces__msg__Object__Sequence * lhs, const zed_interfaces__msg__Object__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zed_interfaces__msg__Object__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zed_interfaces__msg__Object__Sequence__copy(
  const zed_interfaces__msg__Object__Sequence * input,
  zed_interfaces__msg__Object__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zed_interfaces__msg__Object);
    zed_interfaces__msg__Object * data =
      (zed_interfaces__msg__Object *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zed_interfaces__msg__Object__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          zed_interfaces__msg__Object__fini(&data[i]);
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
    if (!zed_interfaces__msg__Object__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
