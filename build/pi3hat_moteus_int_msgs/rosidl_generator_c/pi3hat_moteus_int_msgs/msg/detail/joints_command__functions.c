// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pi3hat_moteus_int_msgs:msg/JointsCommand.idl
// generated code does not contain a copyright notice
#include "pi3hat_moteus_int_msgs/msg/detail/joints_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
// Member `velocity`
// Member `effort`
// Member `kp_scale`
// Member `kd_scale`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pi3hat_moteus_int_msgs__msg__JointsCommand__init(pi3hat_moteus_int_msgs__msg__JointsCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__Sequence__init(&msg->name, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->position, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->velocity, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->effort, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // kp_scale
  if (!rosidl_runtime_c__double__Sequence__init(&msg->kp_scale, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  // kd_scale
  if (!rosidl_runtime_c__double__Sequence__init(&msg->kd_scale, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
    return false;
  }
  return true;
}

void
pi3hat_moteus_int_msgs__msg__JointsCommand__fini(pi3hat_moteus_int_msgs__msg__JointsCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // name
  rosidl_runtime_c__String__Sequence__fini(&msg->name);
  // position
  rosidl_runtime_c__double__Sequence__fini(&msg->position);
  // velocity
  rosidl_runtime_c__double__Sequence__fini(&msg->velocity);
  // effort
  rosidl_runtime_c__double__Sequence__fini(&msg->effort);
  // kp_scale
  rosidl_runtime_c__double__Sequence__fini(&msg->kp_scale);
  // kd_scale
  rosidl_runtime_c__double__Sequence__fini(&msg->kd_scale);
}

bool
pi3hat_moteus_int_msgs__msg__JointsCommand__are_equal(const pi3hat_moteus_int_msgs__msg__JointsCommand * lhs, const pi3hat_moteus_int_msgs__msg__JointsCommand * rhs)
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
  // name
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->effort), &(rhs->effort)))
  {
    return false;
  }
  // kp_scale
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->kp_scale), &(rhs->kp_scale)))
  {
    return false;
  }
  // kd_scale
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->kd_scale), &(rhs->kd_scale)))
  {
    return false;
  }
  return true;
}

bool
pi3hat_moteus_int_msgs__msg__JointsCommand__copy(
  const pi3hat_moteus_int_msgs__msg__JointsCommand * input,
  pi3hat_moteus_int_msgs__msg__JointsCommand * output)
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
  // name
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->effort), &(output->effort)))
  {
    return false;
  }
  // kp_scale
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->kp_scale), &(output->kp_scale)))
  {
    return false;
  }
  // kd_scale
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->kd_scale), &(output->kd_scale)))
  {
    return false;
  }
  return true;
}

pi3hat_moteus_int_msgs__msg__JointsCommand *
pi3hat_moteus_int_msgs__msg__JointsCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pi3hat_moteus_int_msgs__msg__JointsCommand * msg = (pi3hat_moteus_int_msgs__msg__JointsCommand *)allocator.allocate(sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand));
  bool success = pi3hat_moteus_int_msgs__msg__JointsCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pi3hat_moteus_int_msgs__msg__JointsCommand__destroy(pi3hat_moteus_int_msgs__msg__JointsCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__init(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pi3hat_moteus_int_msgs__msg__JointsCommand * data = NULL;

  if (size) {
    data = (pi3hat_moteus_int_msgs__msg__JointsCommand *)allocator.zero_allocate(size, sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pi3hat_moteus_int_msgs__msg__JointsCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pi3hat_moteus_int_msgs__msg__JointsCommand__fini(&data[i - 1]);
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
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__fini(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array)
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
      pi3hat_moteus_int_msgs__msg__JointsCommand__fini(&array->data[i]);
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

pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence *
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array = (pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence *)allocator.allocate(sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__destroy(pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__are_equal(const pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * lhs, const pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pi3hat_moteus_int_msgs__msg__JointsCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence__copy(
  const pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * input,
  pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pi3hat_moteus_int_msgs__msg__JointsCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pi3hat_moteus_int_msgs__msg__JointsCommand * data =
      (pi3hat_moteus_int_msgs__msg__JointsCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pi3hat_moteus_int_msgs__msg__JointsCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pi3hat_moteus_int_msgs__msg__JointsCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pi3hat_moteus_int_msgs__msg__JointsCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
