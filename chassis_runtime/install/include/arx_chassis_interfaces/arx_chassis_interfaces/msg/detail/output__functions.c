// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arx_chassis_interfaces:msg/Output.idl
// generated code does not contain a copyright notice
#include "arx_chassis_interfaces/msg/detail/output__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `scheduler_instance`
// Member `driver_instance`
// Member `reason`
// Member `source_id`
#include "rosidl_runtime_c/string_functions.h"

bool
arx_chassis_interfaces__msg__Output__init(arx_chassis_interfaces__msg__Output * msg)
{
  if (!msg) {
    return false;
  }
  // scheduler_instance
  if (!rosidl_runtime_c__String__init(&msg->scheduler_instance)) {
    arx_chassis_interfaces__msg__Output__fini(msg);
    return false;
  }
  // driver_instance
  if (!rosidl_runtime_c__String__init(&msg->driver_instance)) {
    arx_chassis_interfaces__msg__Output__fini(msg);
    return false;
  }
  // output_seq
  // issued_at
  // ttl_s
  // sdk
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    arx_chassis_interfaces__msg__Output__fini(msg);
    return false;
  }
  // source_id
  if (!rosidl_runtime_c__String__init(&msg->source_id)) {
    arx_chassis_interfaces__msg__Output__fini(msg);
    return false;
  }
  return true;
}

void
arx_chassis_interfaces__msg__Output__fini(arx_chassis_interfaces__msg__Output * msg)
{
  if (!msg) {
    return;
  }
  // scheduler_instance
  rosidl_runtime_c__String__fini(&msg->scheduler_instance);
  // driver_instance
  rosidl_runtime_c__String__fini(&msg->driver_instance);
  // output_seq
  // issued_at
  // ttl_s
  // sdk
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // source_id
  rosidl_runtime_c__String__fini(&msg->source_id);
}

bool
arx_chassis_interfaces__msg__Output__are_equal(const arx_chassis_interfaces__msg__Output * lhs, const arx_chassis_interfaces__msg__Output * rhs)
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
  // driver_instance
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->driver_instance), &(rhs->driver_instance)))
  {
    return false;
  }
  // output_seq
  if (lhs->output_seq != rhs->output_seq) {
    return false;
  }
  // issued_at
  if (lhs->issued_at != rhs->issued_at) {
    return false;
  }
  // ttl_s
  if (lhs->ttl_s != rhs->ttl_s) {
    return false;
  }
  // sdk
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->sdk[i] != rhs->sdk[i]) {
      return false;
    }
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  // source_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->source_id), &(rhs->source_id)))
  {
    return false;
  }
  return true;
}

bool
arx_chassis_interfaces__msg__Output__copy(
  const arx_chassis_interfaces__msg__Output * input,
  arx_chassis_interfaces__msg__Output * output)
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
  // driver_instance
  if (!rosidl_runtime_c__String__copy(
      &(input->driver_instance), &(output->driver_instance)))
  {
    return false;
  }
  // output_seq
  output->output_seq = input->output_seq;
  // issued_at
  output->issued_at = input->issued_at;
  // ttl_s
  output->ttl_s = input->ttl_s;
  // sdk
  for (size_t i = 0; i < 3; ++i) {
    output->sdk[i] = input->sdk[i];
  }
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  // source_id
  if (!rosidl_runtime_c__String__copy(
      &(input->source_id), &(output->source_id)))
  {
    return false;
  }
  return true;
}

arx_chassis_interfaces__msg__Output *
arx_chassis_interfaces__msg__Output__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__msg__Output * msg = (arx_chassis_interfaces__msg__Output *)allocator.allocate(sizeof(arx_chassis_interfaces__msg__Output), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arx_chassis_interfaces__msg__Output));
  bool success = arx_chassis_interfaces__msg__Output__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arx_chassis_interfaces__msg__Output__destroy(arx_chassis_interfaces__msg__Output * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arx_chassis_interfaces__msg__Output__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arx_chassis_interfaces__msg__Output__Sequence__init(arx_chassis_interfaces__msg__Output__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__msg__Output * data = NULL;

  if (size) {
    data = (arx_chassis_interfaces__msg__Output *)allocator.zero_allocate(size, sizeof(arx_chassis_interfaces__msg__Output), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arx_chassis_interfaces__msg__Output__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arx_chassis_interfaces__msg__Output__fini(&data[i - 1]);
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
arx_chassis_interfaces__msg__Output__Sequence__fini(arx_chassis_interfaces__msg__Output__Sequence * array)
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
      arx_chassis_interfaces__msg__Output__fini(&array->data[i]);
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

arx_chassis_interfaces__msg__Output__Sequence *
arx_chassis_interfaces__msg__Output__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__msg__Output__Sequence * array = (arx_chassis_interfaces__msg__Output__Sequence *)allocator.allocate(sizeof(arx_chassis_interfaces__msg__Output__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arx_chassis_interfaces__msg__Output__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arx_chassis_interfaces__msg__Output__Sequence__destroy(arx_chassis_interfaces__msg__Output__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arx_chassis_interfaces__msg__Output__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arx_chassis_interfaces__msg__Output__Sequence__are_equal(const arx_chassis_interfaces__msg__Output__Sequence * lhs, const arx_chassis_interfaces__msg__Output__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arx_chassis_interfaces__msg__Output__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arx_chassis_interfaces__msg__Output__Sequence__copy(
  const arx_chassis_interfaces__msg__Output__Sequence * input,
  arx_chassis_interfaces__msg__Output__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arx_chassis_interfaces__msg__Output);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arx_chassis_interfaces__msg__Output * data =
      (arx_chassis_interfaces__msg__Output *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arx_chassis_interfaces__msg__Output__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arx_chassis_interfaces__msg__Output__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arx_chassis_interfaces__msg__Output__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
