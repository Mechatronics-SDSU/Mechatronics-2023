// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scion_types:srv/SendFrame.idl
// generated code does not contain a copyright notice
#include "scion_types/srv/detail/send_frame__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
scion_types__srv__SendFrame_Request__init(scion_types__srv__SendFrame_Request * msg)
{
  if (!msg) {
    return false;
  }
  // can_id
  // can_dlc
  // can_data
  return true;
}

void
scion_types__srv__SendFrame_Request__fini(scion_types__srv__SendFrame_Request * msg)
{
  if (!msg) {
    return;
  }
  // can_id
  // can_dlc
  // can_data
}

bool
scion_types__srv__SendFrame_Request__are_equal(const scion_types__srv__SendFrame_Request * lhs, const scion_types__srv__SendFrame_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // can_id
  if (lhs->can_id != rhs->can_id) {
    return false;
  }
  // can_dlc
  if (lhs->can_dlc != rhs->can_dlc) {
    return false;
  }
  // can_data
  for (size_t i = 0; i < 8; ++i) {
    if (lhs->can_data[i] != rhs->can_data[i]) {
      return false;
    }
  }
  return true;
}

bool
scion_types__srv__SendFrame_Request__copy(
  const scion_types__srv__SendFrame_Request * input,
  scion_types__srv__SendFrame_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // can_id
  output->can_id = input->can_id;
  // can_dlc
  output->can_dlc = input->can_dlc;
  // can_data
  for (size_t i = 0; i < 8; ++i) {
    output->can_data[i] = input->can_data[i];
  }
  return true;
}

scion_types__srv__SendFrame_Request *
scion_types__srv__SendFrame_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scion_types__srv__SendFrame_Request * msg = (scion_types__srv__SendFrame_Request *)allocator.allocate(sizeof(scion_types__srv__SendFrame_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scion_types__srv__SendFrame_Request));
  bool success = scion_types__srv__SendFrame_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scion_types__srv__SendFrame_Request__destroy(scion_types__srv__SendFrame_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scion_types__srv__SendFrame_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scion_types__srv__SendFrame_Request__Sequence__init(scion_types__srv__SendFrame_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scion_types__srv__SendFrame_Request * data = NULL;

  if (size) {
    data = (scion_types__srv__SendFrame_Request *)allocator.zero_allocate(size, sizeof(scion_types__srv__SendFrame_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scion_types__srv__SendFrame_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scion_types__srv__SendFrame_Request__fini(&data[i - 1]);
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
scion_types__srv__SendFrame_Request__Sequence__fini(scion_types__srv__SendFrame_Request__Sequence * array)
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
      scion_types__srv__SendFrame_Request__fini(&array->data[i]);
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

scion_types__srv__SendFrame_Request__Sequence *
scion_types__srv__SendFrame_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scion_types__srv__SendFrame_Request__Sequence * array = (scion_types__srv__SendFrame_Request__Sequence *)allocator.allocate(sizeof(scion_types__srv__SendFrame_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scion_types__srv__SendFrame_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scion_types__srv__SendFrame_Request__Sequence__destroy(scion_types__srv__SendFrame_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scion_types__srv__SendFrame_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scion_types__srv__SendFrame_Request__Sequence__are_equal(const scion_types__srv__SendFrame_Request__Sequence * lhs, const scion_types__srv__SendFrame_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scion_types__srv__SendFrame_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scion_types__srv__SendFrame_Request__Sequence__copy(
  const scion_types__srv__SendFrame_Request__Sequence * input,
  scion_types__srv__SendFrame_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scion_types__srv__SendFrame_Request);
    scion_types__srv__SendFrame_Request * data =
      (scion_types__srv__SendFrame_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scion_types__srv__SendFrame_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          scion_types__srv__SendFrame_Request__fini(&data[i]);
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
    if (!scion_types__srv__SendFrame_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
scion_types__srv__SendFrame_Response__init(scion_types__srv__SendFrame_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  return true;
}

void
scion_types__srv__SendFrame_Response__fini(scion_types__srv__SendFrame_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
}

bool
scion_types__srv__SendFrame_Response__are_equal(const scion_types__srv__SendFrame_Response * lhs, const scion_types__srv__SendFrame_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
scion_types__srv__SendFrame_Response__copy(
  const scion_types__srv__SendFrame_Response * input,
  scion_types__srv__SendFrame_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  return true;
}

scion_types__srv__SendFrame_Response *
scion_types__srv__SendFrame_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scion_types__srv__SendFrame_Response * msg = (scion_types__srv__SendFrame_Response *)allocator.allocate(sizeof(scion_types__srv__SendFrame_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scion_types__srv__SendFrame_Response));
  bool success = scion_types__srv__SendFrame_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scion_types__srv__SendFrame_Response__destroy(scion_types__srv__SendFrame_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scion_types__srv__SendFrame_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scion_types__srv__SendFrame_Response__Sequence__init(scion_types__srv__SendFrame_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scion_types__srv__SendFrame_Response * data = NULL;

  if (size) {
    data = (scion_types__srv__SendFrame_Response *)allocator.zero_allocate(size, sizeof(scion_types__srv__SendFrame_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scion_types__srv__SendFrame_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scion_types__srv__SendFrame_Response__fini(&data[i - 1]);
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
scion_types__srv__SendFrame_Response__Sequence__fini(scion_types__srv__SendFrame_Response__Sequence * array)
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
      scion_types__srv__SendFrame_Response__fini(&array->data[i]);
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

scion_types__srv__SendFrame_Response__Sequence *
scion_types__srv__SendFrame_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scion_types__srv__SendFrame_Response__Sequence * array = (scion_types__srv__SendFrame_Response__Sequence *)allocator.allocate(sizeof(scion_types__srv__SendFrame_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scion_types__srv__SendFrame_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scion_types__srv__SendFrame_Response__Sequence__destroy(scion_types__srv__SendFrame_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scion_types__srv__SendFrame_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scion_types__srv__SendFrame_Response__Sequence__are_equal(const scion_types__srv__SendFrame_Response__Sequence * lhs, const scion_types__srv__SendFrame_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scion_types__srv__SendFrame_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scion_types__srv__SendFrame_Response__Sequence__copy(
  const scion_types__srv__SendFrame_Response__Sequence * input,
  scion_types__srv__SendFrame_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scion_types__srv__SendFrame_Response);
    scion_types__srv__SendFrame_Response * data =
      (scion_types__srv__SendFrame_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scion_types__srv__SendFrame_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          scion_types__srv__SendFrame_Response__fini(&data[i]);
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
    if (!scion_types__srv__SendFrame_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
