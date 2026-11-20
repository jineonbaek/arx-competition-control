// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arx_chassis_interfaces:msg/State.idl
// generated code does not contain a copyright notice
#include "arx_chassis_interfaces/msg/detail/state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `scheduler_instance`
// Member `driver_instance`
// Member `client_id`
// Member `source_id`
// Member `task_id`
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

bool
arx_chassis_interfaces__msg__State__init(arx_chassis_interfaces__msg__State * msg)
{
  if (!msg) {
    return false;
  }
  // scheduler_instance
  if (!rosidl_runtime_c__String__init(&msg->scheduler_instance)) {
    arx_chassis_interfaces__msg__State__fini(msg);
    return false;
  }
  // ready
  // driver_instance
  if (!rosidl_runtime_c__String__init(&msg->driver_instance)) {
    arx_chassis_interfaces__msg__State__fini(msg);
    return false;
  }
  // active
  // client_id
  if (!rosidl_runtime_c__String__init(&msg->client_id)) {
    arx_chassis_interfaces__msg__State__fini(msg);
    return false;
  }
  // source_id
  if (!rosidl_runtime_c__String__init(&msg->source_id)) {
    arx_chassis_interfaces__msg__State__fini(msg);
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__init(&msg->task_id)) {
    arx_chassis_interfaces__msg__State__fini(msg);
    return false;
  }
  // generation
  // output_seq
  // expires_at
  // sdk
  // maintenance
  // applied_sdk
  // applied_seq
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    arx_chassis_interfaces__msg__State__fini(msg);
    return false;
  }
  return true;
}

void
arx_chassis_interfaces__msg__State__fini(arx_chassis_interfaces__msg__State * msg)
{
  if (!msg) {
    return;
  }
  // scheduler_instance
  rosidl_runtime_c__String__fini(&msg->scheduler_instance);
  // ready
  // driver_instance
  rosidl_runtime_c__String__fini(&msg->driver_instance);
  // active
  // client_id
  rosidl_runtime_c__String__fini(&msg->client_id);
  // source_id
  rosidl_runtime_c__String__fini(&msg->source_id);
  // task_id
  rosidl_runtime_c__String__fini(&msg->task_id);
  // generation
  // output_seq
  // expires_at
  // sdk
  // maintenance
  // applied_sdk
  // applied_seq
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
}

bool
arx_chassis_interfaces__msg__State__are_equal(const arx_chassis_interfaces__msg__State * lhs, const arx_chassis_interfaces__msg__State * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // scheduler_instance
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->scheduler_instance), &(rhs->scheduler_instance)))
  {
    return false;
  }
  // ready
  if (lhs->ready != rhs->ready) {
    return false;
  }
  // driver_instance
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->driver_instance), &(rhs->driver_instance)))
  {
    return false;
  }
  // active
  if (lhs->active != rhs->active) {
    return false;
  }
  // client_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->client_id), &(rhs->client_id)))
  {
    return false;
  }
  // source_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->source_id), &(rhs->source_id)))
  {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_id), &(rhs->task_id)))
  {
    return false;
  }
  // generation
  if (lhs->generation != rhs->generation) {
    return false;
  }
  // output_seq
  if (lhs->output_seq != rhs->output_seq) {
    return false;
  }
  // expires_at
  if (lhs->expires_at != rhs->expires_at) {
    return false;
  }
  // sdk
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->sdk[i] != rhs->sdk[i]) {
      return false;
    }
  }
  // maintenance
  if (lhs->maintenance != rhs->maintenance) {
    return false;
  }
  // applied_sdk
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->applied_sdk[i] != rhs->applied_sdk[i]) {
      return false;
    }
  }
  // applied_seq
  if (lhs->applied_seq != rhs->applied_seq) {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  return true;
}

bool
arx_chassis_interfaces__msg__State__copy(
  const arx_chassis_interfaces__msg__State * input,
  arx_chassis_interfaces__msg__State * output)
{
  if (!input || !output) {
    return false;
  }
  // scheduler_instance
  if (!rosidl_runtime_c__String__copy(
      &(input->scheduler_instance), &(output->scheduler_instance)))
  {
    return false;
  }
  // ready
  output->ready = input->ready;
  // driver_instance
  if (!rosidl_runtime_c__String__copy(
      &(input->driver_instance), &(output->driver_instance)))
  {
    return false;
  }
  // active
  output->active = input->active;
  // client_id
  if (!rosidl_runtime_c__String__copy(
      &(input->client_id), &(output->client_id)))
  {
    return false;
  }
  // source_id
  if (!rosidl_runtime_c__String__copy(
      &(input->source_id), &(output->source_id)))
  {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__copy(
      &(input->task_id), &(output->task_id)))
  {
    return false;
  }
  // generation
  output->generation = input->generation;
  // output_seq
  output->output_seq = input->output_seq;
  // expires_at
  output->expires_at = input->expires_at;
  // sdk
  for (size_t i = 0; i < 3; ++i) {
    output->sdk[i] = input->sdk[i];
  }
  // maintenance
  output->maintenance = input->maintenance;
  // applied_sdk
  for (size_t i = 0; i < 3; ++i) {
    output->applied_sdk[i] = input->applied_sdk[i];
  }
  // applied_seq
  output->applied_seq = input->applied_seq;
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  return true;
}

arx_chassis_interfaces__msg__State *
arx_chassis_interfaces__msg__State__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__msg__State * msg = (arx_chassis_interfaces__msg__State *)allocator.allocate(sizeof(arx_chassis_interfaces__msg__State), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arx_chassis_interfaces__msg__State));
  bool success = arx_chassis_interfaces__msg__State__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arx_chassis_interfaces__msg__State__destroy(arx_chassis_interfaces__msg__State * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arx_chassis_interfaces__msg__State__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arx_chassis_interfaces__msg__State__Sequence__init(arx_chassis_interfaces__msg__State__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__msg__State * data = NULL;

  if (size) {
    data = (arx_chassis_interfaces__msg__State *)allocator.zero_allocate(size, sizeof(arx_chassis_interfaces__msg__State), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arx_chassis_interfaces__msg__State__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arx_chassis_interfaces__msg__State__fini(&data[i - 1]);
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
arx_chassis_interfaces__msg__State__Sequence__fini(arx_chassis_interfaces__msg__State__Sequence * array)
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
      arx_chassis_interfaces__msg__State__fini(&array->data[i]);
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

arx_chassis_interfaces__msg__State__Sequence *
arx_chassis_interfaces__msg__State__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__msg__State__Sequence * array = (arx_chassis_interfaces__msg__State__Sequence *)allocator.allocate(sizeof(arx_chassis_interfaces__msg__State__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arx_chassis_interfaces__msg__State__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arx_chassis_interfaces__msg__State__Sequence__destroy(arx_chassis_interfaces__msg__State__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arx_chassis_interfaces__msg__State__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arx_chassis_interfaces__msg__State__Sequence__are_equal(const arx_chassis_interfaces__msg__State__Sequence * lhs, const arx_chassis_interfaces__msg__State__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arx_chassis_interfaces__msg__State__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arx_chassis_interfaces__msg__State__Sequence__copy(
  const arx_chassis_interfaces__msg__State__Sequence * input,
  arx_chassis_interfaces__msg__State__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arx_chassis_interfaces__msg__State);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arx_chassis_interfaces__msg__State * data =
      (arx_chassis_interfaces__msg__State *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arx_chassis_interfaces__msg__State__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arx_chassis_interfaces__msg__State__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arx_chassis_interfaces__msg__State__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
