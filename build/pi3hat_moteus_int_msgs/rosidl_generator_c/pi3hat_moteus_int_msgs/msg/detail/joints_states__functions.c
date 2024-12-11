// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pi3hat_moteus_int_msgs:msg/JointsStates.idl
// generated code does not contain a copyright notice
#include "pi3hat_moteus_int_msgs/msg/detail/joints_states__functions.h"

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
// Member `current`
// Member `temperature`
// Member `sec_enc_pos`
// Member `sec_enc_vel`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
pi3hat_moteus_int_msgs__msg__JointsStates__init(pi3hat_moteus_int_msgs__msg__JointsStates * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__Sequence__init(&msg->name, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->position, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->velocity, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->effort, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // current
  if (!rosidl_runtime_c__double__Sequence__init(&msg->current, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // temperature
  if (!rosidl_runtime_c__double__Sequence__init(&msg->temperature, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // sec_enc_pos
  if (!rosidl_runtime_c__double__Sequence__init(&msg->sec_enc_pos, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  // sec_enc_vel
  if (!rosidl_runtime_c__double__Sequence__init(&msg->sec_enc_vel, 0)) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
    return false;
  }
  return true;
}

void
pi3hat_moteus_int_msgs__msg__JointsStates__fini(pi3hat_moteus_int_msgs__msg__JointsStates * msg)
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
  // current
  rosidl_runtime_c__double__Sequence__fini(&msg->current);
  // temperature
  rosidl_runtime_c__double__Sequence__fini(&msg->temperature);
  // sec_enc_pos
  rosidl_runtime_c__double__Sequence__fini(&msg->sec_enc_pos);
  // sec_enc_vel
  rosidl_runtime_c__double__Sequence__fini(&msg->sec_enc_vel);
}

bool
pi3hat_moteus_int_msgs__msg__JointsStates__are_equal(const pi3hat_moteus_int_msgs__msg__JointsStates * lhs, const pi3hat_moteus_int_msgs__msg__JointsStates * rhs)
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
  // current
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->current), &(rhs->current)))
  {
    return false;
  }
  // temperature
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->temperature), &(rhs->temperature)))
  {
    return false;
  }
  // sec_enc_pos
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->sec_enc_pos), &(rhs->sec_enc_pos)))
  {
    return false;
  }
  // sec_enc_vel
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->sec_enc_vel), &(rhs->sec_enc_vel)))
  {
    return false;
  }
  return true;
}

bool
pi3hat_moteus_int_msgs__msg__JointsStates__copy(
  const pi3hat_moteus_int_msgs__msg__JointsStates * input,
  pi3hat_moteus_int_msgs__msg__JointsStates * output)
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
  // current
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->current), &(output->current)))
  {
    return false;
  }
  // temperature
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->temperature), &(output->temperature)))
  {
    return false;
  }
  // sec_enc_pos
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->sec_enc_pos), &(output->sec_enc_pos)))
  {
    return false;
  }
  // sec_enc_vel
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->sec_enc_vel), &(output->sec_enc_vel)))
  {
    return false;
  }
  return true;
}

pi3hat_moteus_int_msgs__msg__JointsStates *
pi3hat_moteus_int_msgs__msg__JointsStates__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pi3hat_moteus_int_msgs__msg__JointsStates * msg = (pi3hat_moteus_int_msgs__msg__JointsStates *)allocator.allocate(sizeof(pi3hat_moteus_int_msgs__msg__JointsStates), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pi3hat_moteus_int_msgs__msg__JointsStates));
  bool success = pi3hat_moteus_int_msgs__msg__JointsStates__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pi3hat_moteus_int_msgs__msg__JointsStates__destroy(pi3hat_moteus_int_msgs__msg__JointsStates * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pi3hat_moteus_int_msgs__msg__JointsStates__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__init(pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pi3hat_moteus_int_msgs__msg__JointsStates * data = NULL;

  if (size) {
    data = (pi3hat_moteus_int_msgs__msg__JointsStates *)allocator.zero_allocate(size, sizeof(pi3hat_moteus_int_msgs__msg__JointsStates), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pi3hat_moteus_int_msgs__msg__JointsStates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pi3hat_moteus_int_msgs__msg__JointsStates__fini(&data[i - 1]);
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
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__fini(pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * array)
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
      pi3hat_moteus_int_msgs__msg__JointsStates__fini(&array->data[i]);
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

pi3hat_moteus_int_msgs__msg__JointsStates__Sequence *
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * array = (pi3hat_moteus_int_msgs__msg__JointsStates__Sequence *)allocator.allocate(sizeof(pi3hat_moteus_int_msgs__msg__JointsStates__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__destroy(pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__are_equal(const pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * lhs, const pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pi3hat_moteus_int_msgs__msg__JointsStates__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pi3hat_moteus_int_msgs__msg__JointsStates__Sequence__copy(
  const pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * input,
  pi3hat_moteus_int_msgs__msg__JointsStates__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pi3hat_moteus_int_msgs__msg__JointsStates);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pi3hat_moteus_int_msgs__msg__JointsStates * data =
      (pi3hat_moteus_int_msgs__msg__JointsStates *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pi3hat_moteus_int_msgs__msg__JointsStates__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pi3hat_moteus_int_msgs__msg__JointsStates__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pi3hat_moteus_int_msgs__msg__JointsStates__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
