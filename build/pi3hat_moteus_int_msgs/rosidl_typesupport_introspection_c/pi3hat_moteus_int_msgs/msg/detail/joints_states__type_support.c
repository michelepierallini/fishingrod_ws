// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pi3hat_moteus_int_msgs:msg/JointsStates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pi3hat_moteus_int_msgs/msg/detail/joints_states__rosidl_typesupport_introspection_c.h"
#include "pi3hat_moteus_int_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pi3hat_moteus_int_msgs/msg/detail/joints_states__functions.h"
#include "pi3hat_moteus_int_msgs/msg/detail/joints_states__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
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

#ifdef __cplusplus
extern "C"
{
#endif

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pi3hat_moteus_int_msgs__msg__JointsStates__init(message_memory);
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_fini_function(void * message_memory)
{
  pi3hat_moteus_int_msgs__msg__JointsStates__fini(message_memory);
}

size_t pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__name(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__name(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__name(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__name(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__name(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__name(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__effort(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__effort(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__effort(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__effort(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__current(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__current(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__current(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__current(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__current(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__current(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__current(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__current(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__temperature(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__temperature(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__temperature(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__temperature(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__temperature(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__temperature(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__temperature(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__temperature(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__sec_enc_pos(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__sec_enc_pos(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__sec_enc_pos(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__sec_enc_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__sec_enc_pos(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__sec_enc_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__sec_enc_pos(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__sec_enc_pos(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__sec_enc_vel(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__sec_enc_vel(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__sec_enc_vel(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__sec_enc_vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__sec_enc_vel(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__sec_enc_vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__sec_enc_vel(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__sec_enc_vel(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsStates, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsStates, name),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__name,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__name,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__name,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__name,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__name,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__name  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsStates, position),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__position,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__position,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__position,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__position,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__position,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsStates, velocity),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__velocity,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__velocity,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__velocity,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__velocity,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__velocity,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__velocity  // resize(index) function pointer
  },
  {
    "effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsStates, effort),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__effort,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__effort,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__effort,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__effort,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__effort,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__effort  // resize(index) function pointer
  },
  {
    "current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsStates, current),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__current,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__current,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__current,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__current,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__current,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__current  // resize(index) function pointer
  },
  {
    "temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsStates, temperature),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__temperature,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__temperature,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__temperature,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__temperature,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__temperature,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__temperature  // resize(index) function pointer
  },
  {
    "sec_enc_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsStates, sec_enc_pos),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__sec_enc_pos,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__sec_enc_pos,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__sec_enc_pos,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__sec_enc_pos,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__sec_enc_pos,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__sec_enc_pos  // resize(index) function pointer
  },
  {
    "sec_enc_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__JointsStates, sec_enc_vel),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__size_function__JointsStates__sec_enc_vel,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_const_function__JointsStates__sec_enc_vel,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__get_function__JointsStates__sec_enc_vel,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__fetch_function__JointsStates__sec_enc_vel,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__assign_function__JointsStates__sec_enc_vel,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__resize_function__JointsStates__sec_enc_vel  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_message_members = {
  "pi3hat_moteus_int_msgs__msg",  // message namespace
  "JointsStates",  // message name
  9,  // number of fields
  sizeof(pi3hat_moteus_int_msgs__msg__JointsStates),
  pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_message_member_array,  // message members
  pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_init_function,  // function to initialize message memory (memory has to be allocated)
  pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_message_type_support_handle = {
  0,
  &pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pi3hat_moteus_int_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pi3hat_moteus_int_msgs, msg, JointsStates)() {
  pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_message_type_support_handle.typesupport_identifier) {
    pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pi3hat_moteus_int_msgs__msg__JointsStates__rosidl_typesupport_introspection_c__JointsStates_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
