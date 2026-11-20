// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arx_chassis_interfaces:srv/Manage.idl
// generated code does not contain a copyright notice
#include "arx_chassis_interfaces/srv/detail/manage__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `operation`
// Member `scheduler_instance`
// Member `client_id`
// Member `profile`
// Member `resume_key`
#include "rosidl_runtime_c/string_functions.h"

bool
arx_chassis_interfaces__srv__Manage_Request__init(arx_chassis_interfaces__srv__Manage_Request * msg)
{
  if (!msg) {
    return false;
  }
  // operation
  if (!rosidl_runtime_c__String__init(&msg->operation)) {
    arx_chassis_interfaces__srv__Manage_Request__fini(msg);
    return false;
  }
  // scheduler_instance
  if (!rosidl_runtime_c__String__init(&msg->scheduler_instance)) {
    arx_chassis_interfaces__srv__Manage_Request__fini(msg);
    return false;
  }
  // client_id
  if (!rosidl_runtime_c__String__init(&msg->client_id)) {
    arx_chassis_interfaces__srv__Manage_Request__fini(msg);
    return false;
  }
  // profile
  if (!rosidl_runtime_c__String__init(&msg->profile)) {
    arx_chassis_interfaces__srv__Manage_Request__fini(msg);
    return false;
  }
  // resume_key
  if (!rosidl_runtime_c__String__init(&msg->resume_key)) {
    arx_chassis_interfaces__srv__Manage_Request__fini(msg);
    return false;
  }
  // output_seq
  return true;
}

void
arx_chassis_interfaces__srv__Manage_Request__fini(arx_chassis_interfaces__srv__Manage_Request * msg)
{
  if (!msg) {
    return;
  }
  // operation
  rosidl_runtime_c__String__fini(&msg->operation);
  // scheduler_instance
  rosidl_runtime_c__String__fini(&msg->scheduler_instance);
  // client_id
  rosidl_runtime_c__String__fini(&msg->client_id);
  // profile
  rosidl_runtime_c__String__fini(&msg->profile);
  // resume_key
  rosidl_runtime_c__String__fini(&msg->resume_key);
  // output_seq
}

bool
arx_chassis_interfaces__srv__Manage_Request__are_equal(const arx_chassis_interfaces__srv__Manage_Request * lhs, const arx_chassis_interfaces__srv__Manage_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // operation
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->operation), &(rhs->operation)))
  {
    return false;
  }
  // scheduler_instance
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->scheduler_instance), &(rhs->scheduler_instance)))
  {
    return false;
  }
  // client_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->client_id), &(rhs->client_id)))
  {
    return false;
  }
  // profile
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->profile), &(rhs->profile)))
  {
    return false;
  }
  // resume_key
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->resume_key), &(rhs->resume_key)))
  {
    return false;
  }
  // output_seq
  if (lhs->output_seq != rhs->output_seq) {
    return false;
  }
  return true;
}

bool
arx_chassis_interfaces__srv__Manage_Request__copy(
  const arx_chassis_interfaces__srv__Manage_Request * input,
  arx_chassis_interfaces__srv__Manage_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // operation
  if (!rosidl_runtime_c__String__copy(
      &(input->operation), &(output->operation)))
  {
    return false;
  }
  // scheduler_instance
  if (!rosidl_runtime_c__String__copy(
      &(input->scheduler_instance), &(output->scheduler_instance)))
  {
    return false;
  }
  // client_id
  if (!rosidl_runtime_c__String__copy(
      &(input->client_id), &(output->client_id)))
  {
    return false;
  }
  // profile
  if (!rosidl_runtime_c__String__copy(
      &(input->profile), &(output->profile)))
  {
    return false;
  }
  // resume_key
  if (!rosidl_runtime_c__String__copy(
      &(input->resume_key), &(output->resume_key)))
  {
    return false;
  }
  // output_seq
  output->output_seq = input->output_seq;
  return true;
}

arx_chassis_interfaces__srv__Manage_Request *
arx_chassis_interfaces__srv__Manage_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__srv__Manage_Request * msg = (arx_chassis_interfaces__srv__Manage_Request *)allocator.allocate(sizeof(arx_chassis_interfaces__srv__Manage_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arx_chassis_interfaces__srv__Manage_Request));
  bool success = arx_chassis_interfaces__srv__Manage_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arx_chassis_interfaces__srv__Manage_Request__destroy(arx_chassis_interfaces__srv__Manage_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arx_chassis_interfaces__srv__Manage_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arx_chassis_interfaces__srv__Manage_Request__Sequence__init(arx_chassis_interfaces__srv__Manage_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__srv__Manage_Request * data = NULL;

  if (size) {
    data = (arx_chassis_interfaces__srv__Manage_Request *)allocator.zero_allocate(size, sizeof(arx_chassis_interfaces__srv__Manage_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arx_chassis_interfaces__srv__Manage_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arx_chassis_interfaces__srv__Manage_Request__fini(&data[i - 1]);
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
arx_chassis_interfaces__srv__Manage_Request__Sequence__fini(arx_chassis_interfaces__srv__Manage_Request__Sequence * array)
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
      arx_chassis_interfaces__srv__Manage_Request__fini(&array->data[i]);
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

arx_chassis_interfaces__srv__Manage_Request__Sequence *
arx_chassis_interfaces__srv__Manage_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__srv__Manage_Request__Sequence * array = (arx_chassis_interfaces__srv__Manage_Request__Sequence *)allocator.allocate(sizeof(arx_chassis_interfaces__srv__Manage_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arx_chassis_interfaces__srv__Manage_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arx_chassis_interfaces__srv__Manage_Request__Sequence__destroy(arx_chassis_interfaces__srv__Manage_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arx_chassis_interfaces__srv__Manage_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arx_chassis_interfaces__srv__Manage_Request__Sequence__are_equal(const arx_chassis_interfaces__srv__Manage_Request__Sequence * lhs, const arx_chassis_interfaces__srv__Manage_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arx_chassis_interfaces__srv__Manage_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arx_chassis_interfaces__srv__Manage_Request__Sequence__copy(
  const arx_chassis_interfaces__srv__Manage_Request__Sequence * input,
  arx_chassis_interfaces__srv__Manage_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arx_chassis_interfaces__srv__Manage_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arx_chassis_interfaces__srv__Manage_Request * data =
      (arx_chassis_interfaces__srv__Manage_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arx_chassis_interfaces__srv__Manage_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arx_chassis_interfaces__srv__Manage_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arx_chassis_interfaces__srv__Manage_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `reason`
// Member `client_id`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `state`
#include "arx_chassis_interfaces/msg/detail/state__functions.h"

bool
arx_chassis_interfaces__srv__Manage_Response__init(arx_chassis_interfaces__srv__Manage_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    arx_chassis_interfaces__srv__Manage_Response__fini(msg);
    return false;
  }
  // client_id
  if (!rosidl_runtime_c__String__init(&msg->client_id)) {
    arx_chassis_interfaces__srv__Manage_Response__fini(msg);
    return false;
  }
  // state
  if (!arx_chassis_interfaces__msg__State__init(&msg->state)) {
    arx_chassis_interfaces__srv__Manage_Response__fini(msg);
    return false;
  }
  return true;
}

void
arx_chassis_interfaces__srv__Manage_Response__fini(arx_chassis_interfaces__srv__Manage_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // client_id
  rosidl_runtime_c__String__fini(&msg->client_id);
  // state
  arx_chassis_interfaces__msg__State__fini(&msg->state);
}

bool
arx_chassis_interfaces__srv__Manage_Response__are_equal(const arx_chassis_interfaces__srv__Manage_Response * lhs, const arx_chassis_interfaces__srv__Manage_Response * rhs)
{
  if (!lhs || !rhs) {
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
  // client_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->client_id), &(rhs->client_id)))
  {
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
arx_chassis_interfaces__srv__Manage_Response__copy(
  const arx_chassis_interfaces__srv__Manage_Response * input,
  arx_chassis_interfaces__srv__Manage_Response * output)
{
  if (!input || !output) {
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
  // client_id
  if (!rosidl_runtime_c__String__copy(
      &(input->client_id), &(output->client_id)))
  {
    return false;
  }
  // state
  if (!arx_chassis_interfaces__msg__State__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  return true;
}

arx_chassis_interfaces__srv__Manage_Response *
arx_chassis_interfaces__srv__Manage_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__srv__Manage_Response * msg = (arx_chassis_interfaces__srv__Manage_Response *)allocator.allocate(sizeof(arx_chassis_interfaces__srv__Manage_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arx_chassis_interfaces__srv__Manage_Response));
  bool success = arx_chassis_interfaces__srv__Manage_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arx_chassis_interfaces__srv__Manage_Response__destroy(arx_chassis_interfaces__srv__Manage_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arx_chassis_interfaces__srv__Manage_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arx_chassis_interfaces__srv__Manage_Response__Sequence__init(arx_chassis_interfaces__srv__Manage_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__srv__Manage_Response * data = NULL;

  if (size) {
    data = (arx_chassis_interfaces__srv__Manage_Response *)allocator.zero_allocate(size, sizeof(arx_chassis_interfaces__srv__Manage_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arx_chassis_interfaces__srv__Manage_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arx_chassis_interfaces__srv__Manage_Response__fini(&data[i - 1]);
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
arx_chassis_interfaces__srv__Manage_Response__Sequence__fini(arx_chassis_interfaces__srv__Manage_Response__Sequence * array)
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
      arx_chassis_interfaces__srv__Manage_Response__fini(&array->data[i]);
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

arx_chassis_interfaces__srv__Manage_Response__Sequence *
arx_chassis_interfaces__srv__Manage_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__srv__Manage_Response__Sequence * array = (arx_chassis_interfaces__srv__Manage_Response__Sequence *)allocator.allocate(sizeof(arx_chassis_interfaces__srv__Manage_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arx_chassis_interfaces__srv__Manage_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arx_chassis_interfaces__srv__Manage_Response__Sequence__destroy(arx_chassis_interfaces__srv__Manage_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arx_chassis_interfaces__srv__Manage_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arx_chassis_interfaces__srv__Manage_Response__Sequence__are_equal(const arx_chassis_interfaces__srv__Manage_Response__Sequence * lhs, const arx_chassis_interfaces__srv__Manage_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arx_chassis_interfaces__srv__Manage_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arx_chassis_interfaces__srv__Manage_Response__Sequence__copy(
  const arx_chassis_interfaces__srv__Manage_Response__Sequence * input,
  arx_chassis_interfaces__srv__Manage_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arx_chassis_interfaces__srv__Manage_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arx_chassis_interfaces__srv__Manage_Response * data =
      (arx_chassis_interfaces__srv__Manage_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arx_chassis_interfaces__srv__Manage_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arx_chassis_interfaces__srv__Manage_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arx_chassis_interfaces__srv__Manage_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "arx_chassis_interfaces/srv/detail/manage__functions.h"

bool
arx_chassis_interfaces__srv__Manage_Event__init(arx_chassis_interfaces__srv__Manage_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    arx_chassis_interfaces__srv__Manage_Event__fini(msg);
    return false;
  }
  // request
  if (!arx_chassis_interfaces__srv__Manage_Request__Sequence__init(&msg->request, 0)) {
    arx_chassis_interfaces__srv__Manage_Event__fini(msg);
    return false;
  }
  // response
  if (!arx_chassis_interfaces__srv__Manage_Response__Sequence__init(&msg->response, 0)) {
    arx_chassis_interfaces__srv__Manage_Event__fini(msg);
    return false;
  }
  return true;
}

void
arx_chassis_interfaces__srv__Manage_Event__fini(arx_chassis_interfaces__srv__Manage_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  arx_chassis_interfaces__srv__Manage_Request__Sequence__fini(&msg->request);
  // response
  arx_chassis_interfaces__srv__Manage_Response__Sequence__fini(&msg->response);
}

bool
arx_chassis_interfaces__srv__Manage_Event__are_equal(const arx_chassis_interfaces__srv__Manage_Event * lhs, const arx_chassis_interfaces__srv__Manage_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!arx_chassis_interfaces__srv__Manage_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!arx_chassis_interfaces__srv__Manage_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
arx_chassis_interfaces__srv__Manage_Event__copy(
  const arx_chassis_interfaces__srv__Manage_Event * input,
  arx_chassis_interfaces__srv__Manage_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!arx_chassis_interfaces__srv__Manage_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!arx_chassis_interfaces__srv__Manage_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

arx_chassis_interfaces__srv__Manage_Event *
arx_chassis_interfaces__srv__Manage_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__srv__Manage_Event * msg = (arx_chassis_interfaces__srv__Manage_Event *)allocator.allocate(sizeof(arx_chassis_interfaces__srv__Manage_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arx_chassis_interfaces__srv__Manage_Event));
  bool success = arx_chassis_interfaces__srv__Manage_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arx_chassis_interfaces__srv__Manage_Event__destroy(arx_chassis_interfaces__srv__Manage_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arx_chassis_interfaces__srv__Manage_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arx_chassis_interfaces__srv__Manage_Event__Sequence__init(arx_chassis_interfaces__srv__Manage_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__srv__Manage_Event * data = NULL;

  if (size) {
    data = (arx_chassis_interfaces__srv__Manage_Event *)allocator.zero_allocate(size, sizeof(arx_chassis_interfaces__srv__Manage_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arx_chassis_interfaces__srv__Manage_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arx_chassis_interfaces__srv__Manage_Event__fini(&data[i - 1]);
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
arx_chassis_interfaces__srv__Manage_Event__Sequence__fini(arx_chassis_interfaces__srv__Manage_Event__Sequence * array)
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
      arx_chassis_interfaces__srv__Manage_Event__fini(&array->data[i]);
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

arx_chassis_interfaces__srv__Manage_Event__Sequence *
arx_chassis_interfaces__srv__Manage_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arx_chassis_interfaces__srv__Manage_Event__Sequence * array = (arx_chassis_interfaces__srv__Manage_Event__Sequence *)allocator.allocate(sizeof(arx_chassis_interfaces__srv__Manage_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arx_chassis_interfaces__srv__Manage_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arx_chassis_interfaces__srv__Manage_Event__Sequence__destroy(arx_chassis_interfaces__srv__Manage_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arx_chassis_interfaces__srv__Manage_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arx_chassis_interfaces__srv__Manage_Event__Sequence__are_equal(const arx_chassis_interfaces__srv__Manage_Event__Sequence * lhs, const arx_chassis_interfaces__srv__Manage_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arx_chassis_interfaces__srv__Manage_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arx_chassis_interfaces__srv__Manage_Event__Sequence__copy(
  const arx_chassis_interfaces__srv__Manage_Event__Sequence * input,
  arx_chassis_interfaces__srv__Manage_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arx_chassis_interfaces__srv__Manage_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arx_chassis_interfaces__srv__Manage_Event * data =
      (arx_chassis_interfaces__srv__Manage_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arx_chassis_interfaces__srv__Manage_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arx_chassis_interfaces__srv__Manage_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arx_chassis_interfaces__srv__Manage_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
