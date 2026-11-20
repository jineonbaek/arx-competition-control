// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arx_chassis_interfaces:msg/Result.idl
// generated code does not contain a copyright notice
#include "arx_chassis_interfaces/msg/detail/result__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `request_id`
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"
// Member `state`
#include "arx_chassis_interfaces/msg/detail/state__functions.h"

bool
arx_chassis_interfaces__msg__Result__init(arx_chassis_interfaces__msg__Result * msg)
{
  if (!msg) {
    return false;
  }
  // request_id
  if (!rosidl_runtime_c__String__init(&msg->request_id)) {
    arx_chassis_interfaces__msg__Result__fini(msg);
    return false;
  }
  // accepted
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    arx_chassis_interfaces__msg__Result__fini(msg);
    return false;
  }
  // output_seq
  // state
  if (!arx_chassis_interfaces__msg__State__init(&msg->state)) {
    arx_chassis_interfaces__msg__Result__fini(msg);
    return false;
  }
  return true;
}

void
arx_chassis_interfaces__msg__Result__fini(arx_chassis_interfaces__msg__Result * msg)
{
  if (!msg) {
    return;
  }
  // request_id
  rosidl_runtime_c__String__fini(&msg->request_id);
  // accepted
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // output_seq
  // state
  arx_chassis_interfaces__msg__State__fini(&msg->state);
}

bool
arx_chassis_interfaces__msg__Result__are_equal(const arx_chassis_interfaces__msg__Result * lhs, const arx_chassis_interfaces__msg__Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // request_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->request_id), &(rhs->request_id)))
  {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  // output_seq
  if (lhs->output_seq != rhs->output_seq) {
    return false;
  }
  // state
  if (!arx_chassis_interfaces__msg__State__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  return true;
}

bool
arx_chassis_interfaces__msg__Result__copy(
  const arx_chassis_interfaces__msg__Result * input,
  arx_chassis_interfaces__msg__Result * output)
{
  if (!input || !output) {
    return false;
  }
  // request_id
  if (!rosidl_runtime_c__String__copy(
      &(input->request_id), &(output->request_id)))
  {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  // output_seq
  output->output_seq = input->output_seq;
  // state
  if (!arx_chassis_interfaces__msg__State__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  return true;
}

arx_chassis_interfaces__msg__Result *
arx_chassis_interfaces__msg__Result__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__msg__Result * msg = (arx_chassis_interfaces__msg__Result *)allocator.allocate(sizeof(arx_chassis_interfaces__msg__Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arx_chassis_interfaces__msg__Result));
  bool success = arx_chassis_interfaces__msg__Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arx_chassis_interfaces__msg__Result__destroy(arx_chassis_interfaces__msg__Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arx_chassis_interfaces__msg__Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arx_chassis_interfaces__msg__Result__Sequence__init(arx_chassis_interfaces__msg__Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__msg__Result * data = NULL;

  if (size) {
    data = (arx_chassis_interfaces__msg__Result *)allocator.zero_allocate(size, sizeof(arx_chassis_interfaces__msg__Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arx_chassis_interfaces__msg__Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arx_chassis_interfaces__msg__Result__fini(&data[i - 1]);
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
arx_chassis_interfaces__msg__Result__Sequence__fini(arx_chassis_interfaces__msg__Result__Sequence * array)
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
      arx_chassis_interfaces__msg__Result__fini(&array->data[i]);
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

arx_chassis_interfaces__msg__Result__Sequence *
arx_chassis_interfaces__msg__Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__msg__Result__Sequence * array = (arx_chassis_interfaces__msg__Result__Sequence *)allocator.allocate(sizeof(arx_chassis_interfaces__msg__Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arx_chassis_interfaces__msg__Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arx_chassis_interfaces__msg__Result__Sequence__destroy(arx_chassis_interfaces__msg__Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arx_chassis_interfaces__msg__Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arx_chassis_interfaces__msg__Result__Sequence__are_equal(const arx_chassis_interfaces__msg__Result__Sequence * lhs, const arx_chassis_interfaces__msg__Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arx_chassis_interfaces__msg__Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arx_chassis_interfaces__msg__Result__Sequence__copy(
  const arx_chassis_interfaces__msg__Result__Sequence * input,
  arx_chassis_interfaces__msg__Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arx_chassis_interfaces__msg__Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arx_chassis_interfaces__msg__Result * data =
      (arx_chassis_interfaces__msg__Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arx_chassis_interfaces__msg__Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arx_chassis_interfaces__msg__Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arx_chassis_interfaces__msg__Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
