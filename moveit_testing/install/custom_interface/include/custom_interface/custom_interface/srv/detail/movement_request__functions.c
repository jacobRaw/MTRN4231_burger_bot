// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interface:srv/MovementRequest.idl
// generated code does not contain a copyright notice
#include "custom_interface/srv/detail/movement_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `command`
// Member `constraints_identifier`
#include "rosidl_runtime_c/string_functions.h"
// Member `positions`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
custom_interface__srv__MovementRequest_Request__init(custom_interface__srv__MovementRequest_Request * msg)
{
  if (!msg) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    custom_interface__srv__MovementRequest_Request__fini(msg);
    return false;
  }
  // positions
  if (!rosidl_runtime_c__double__Sequence__init(&msg->positions, 0)) {
    custom_interface__srv__MovementRequest_Request__fini(msg);
    return false;
  }
  // constraints_identifier
  if (!rosidl_runtime_c__String__init(&msg->constraints_identifier)) {
    custom_interface__srv__MovementRequest_Request__fini(msg);
    return false;
  }
  return true;
}

void
custom_interface__srv__MovementRequest_Request__fini(custom_interface__srv__MovementRequest_Request * msg)
{
  if (!msg) {
    return;
  }
  // command
  rosidl_runtime_c__String__fini(&msg->command);
  // positions
  rosidl_runtime_c__double__Sequence__fini(&msg->positions);
  // constraints_identifier
  rosidl_runtime_c__String__fini(&msg->constraints_identifier);
}

bool
custom_interface__srv__MovementRequest_Request__are_equal(const custom_interface__srv__MovementRequest_Request * lhs, const custom_interface__srv__MovementRequest_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  // positions
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->positions), &(rhs->positions)))
  {
    return false;
  }
  // constraints_identifier
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->constraints_identifier), &(rhs->constraints_identifier)))
  {
    return false;
  }
  return true;
}

bool
custom_interface__srv__MovementRequest_Request__copy(
  const custom_interface__srv__MovementRequest_Request * input,
  custom_interface__srv__MovementRequest_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  // positions
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->positions), &(output->positions)))
  {
    return false;
  }
  // constraints_identifier
  if (!rosidl_runtime_c__String__copy(
      &(input->constraints_identifier), &(output->constraints_identifier)))
  {
    return false;
  }
  return true;
}

custom_interface__srv__MovementRequest_Request *
custom_interface__srv__MovementRequest_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__srv__MovementRequest_Request * msg = (custom_interface__srv__MovementRequest_Request *)allocator.allocate(sizeof(custom_interface__srv__MovementRequest_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interface__srv__MovementRequest_Request));
  bool success = custom_interface__srv__MovementRequest_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interface__srv__MovementRequest_Request__destroy(custom_interface__srv__MovementRequest_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interface__srv__MovementRequest_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interface__srv__MovementRequest_Request__Sequence__init(custom_interface__srv__MovementRequest_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__srv__MovementRequest_Request * data = NULL;

  if (size) {
    data = (custom_interface__srv__MovementRequest_Request *)allocator.zero_allocate(size, sizeof(custom_interface__srv__MovementRequest_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interface__srv__MovementRequest_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interface__srv__MovementRequest_Request__fini(&data[i - 1]);
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
custom_interface__srv__MovementRequest_Request__Sequence__fini(custom_interface__srv__MovementRequest_Request__Sequence * array)
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
      custom_interface__srv__MovementRequest_Request__fini(&array->data[i]);
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

custom_interface__srv__MovementRequest_Request__Sequence *
custom_interface__srv__MovementRequest_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__srv__MovementRequest_Request__Sequence * array = (custom_interface__srv__MovementRequest_Request__Sequence *)allocator.allocate(sizeof(custom_interface__srv__MovementRequest_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interface__srv__MovementRequest_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interface__srv__MovementRequest_Request__Sequence__destroy(custom_interface__srv__MovementRequest_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interface__srv__MovementRequest_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interface__srv__MovementRequest_Request__Sequence__are_equal(const custom_interface__srv__MovementRequest_Request__Sequence * lhs, const custom_interface__srv__MovementRequest_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interface__srv__MovementRequest_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interface__srv__MovementRequest_Request__Sequence__copy(
  const custom_interface__srv__MovementRequest_Request__Sequence * input,
  custom_interface__srv__MovementRequest_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interface__srv__MovementRequest_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interface__srv__MovementRequest_Request * data =
      (custom_interface__srv__MovementRequest_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interface__srv__MovementRequest_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interface__srv__MovementRequest_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interface__srv__MovementRequest_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
custom_interface__srv__MovementRequest_Response__init(custom_interface__srv__MovementRequest_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
custom_interface__srv__MovementRequest_Response__fini(custom_interface__srv__MovementRequest_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
custom_interface__srv__MovementRequest_Response__are_equal(const custom_interface__srv__MovementRequest_Response * lhs, const custom_interface__srv__MovementRequest_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
custom_interface__srv__MovementRequest_Response__copy(
  const custom_interface__srv__MovementRequest_Response * input,
  custom_interface__srv__MovementRequest_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

custom_interface__srv__MovementRequest_Response *
custom_interface__srv__MovementRequest_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__srv__MovementRequest_Response * msg = (custom_interface__srv__MovementRequest_Response *)allocator.allocate(sizeof(custom_interface__srv__MovementRequest_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interface__srv__MovementRequest_Response));
  bool success = custom_interface__srv__MovementRequest_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interface__srv__MovementRequest_Response__destroy(custom_interface__srv__MovementRequest_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interface__srv__MovementRequest_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interface__srv__MovementRequest_Response__Sequence__init(custom_interface__srv__MovementRequest_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__srv__MovementRequest_Response * data = NULL;

  if (size) {
    data = (custom_interface__srv__MovementRequest_Response *)allocator.zero_allocate(size, sizeof(custom_interface__srv__MovementRequest_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interface__srv__MovementRequest_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interface__srv__MovementRequest_Response__fini(&data[i - 1]);
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
custom_interface__srv__MovementRequest_Response__Sequence__fini(custom_interface__srv__MovementRequest_Response__Sequence * array)
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
      custom_interface__srv__MovementRequest_Response__fini(&array->data[i]);
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

custom_interface__srv__MovementRequest_Response__Sequence *
custom_interface__srv__MovementRequest_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__srv__MovementRequest_Response__Sequence * array = (custom_interface__srv__MovementRequest_Response__Sequence *)allocator.allocate(sizeof(custom_interface__srv__MovementRequest_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interface__srv__MovementRequest_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interface__srv__MovementRequest_Response__Sequence__destroy(custom_interface__srv__MovementRequest_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interface__srv__MovementRequest_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interface__srv__MovementRequest_Response__Sequence__are_equal(const custom_interface__srv__MovementRequest_Response__Sequence * lhs, const custom_interface__srv__MovementRequest_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interface__srv__MovementRequest_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interface__srv__MovementRequest_Response__Sequence__copy(
  const custom_interface__srv__MovementRequest_Response__Sequence * input,
  custom_interface__srv__MovementRequest_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interface__srv__MovementRequest_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interface__srv__MovementRequest_Response * data =
      (custom_interface__srv__MovementRequest_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interface__srv__MovementRequest_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interface__srv__MovementRequest_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interface__srv__MovementRequest_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
