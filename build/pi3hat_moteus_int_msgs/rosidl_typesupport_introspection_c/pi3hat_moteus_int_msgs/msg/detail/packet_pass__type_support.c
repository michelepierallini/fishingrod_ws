// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pi3hat_moteus_int_msgs:msg/PacketPass.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pi3hat_moteus_int_msgs/msg/detail/packet_pass__rosidl_typesupport_introspection_c.h"
#include "pi3hat_moteus_int_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pi3hat_moteus_int_msgs/msg/detail/packet_pass__functions.h"
#include "pi3hat_moteus_int_msgs/msg/detail/packet_pass__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `pack_loss`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pi3hat_moteus_int_msgs__msg__PacketPass__init(message_memory);
}

void pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_fini_function(void * message_memory)
{
  pi3hat_moteus_int_msgs__msg__PacketPass__fini(message_memory);
}

size_t pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__size_function__PacketPass__name(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_const_function__PacketPass__name(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_function__PacketPass__name(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__fetch_function__PacketPass__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_const_function__PacketPass__name(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__assign_function__PacketPass__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_function__PacketPass__name(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__resize_function__PacketPass__name(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__size_function__PacketPass__pack_loss(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_const_function__PacketPass__pack_loss(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_function__PacketPass__pack_loss(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__fetch_function__PacketPass__pack_loss(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_const_function__PacketPass__pack_loss(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__assign_function__PacketPass__pack_loss(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_function__PacketPass__pack_loss(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__resize_function__PacketPass__pack_loss(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cycle_dur",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, cycle_dur),  // bytes offset in struct
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
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, name),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__size_function__PacketPass__name,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_const_function__PacketPass__name,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_function__PacketPass__name,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__fetch_function__PacketPass__name,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__assign_function__PacketPass__name,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__resize_function__PacketPass__name  // resize(index) function pointer
  },
  {
    "pack_loss",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pi3hat_moteus_int_msgs__msg__PacketPass, pack_loss),  // bytes offset in struct
    NULL,  // default value
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__size_function__PacketPass__pack_loss,  // size() function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_const_function__PacketPass__pack_loss,  // get_const(index) function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__get_function__PacketPass__pack_loss,  // get(index) function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__fetch_function__PacketPass__pack_loss,  // fetch(index, &value) function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__assign_function__PacketPass__pack_loss,  // assign(index, value) function pointer
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__resize_function__PacketPass__pack_loss  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_members = {
  "pi3hat_moteus_int_msgs__msg",  // message namespace
  "PacketPass",  // message name
  5,  // number of fields
  sizeof(pi3hat_moteus_int_msgs__msg__PacketPass),
  pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_member_array,  // message members
  pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_init_function,  // function to initialize message memory (memory has to be allocated)
  pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_type_support_handle = {
  0,
  &pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pi3hat_moteus_int_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pi3hat_moteus_int_msgs, msg, PacketPass)() {
  pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_type_support_handle.typesupport_identifier) {
    pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pi3hat_moteus_int_msgs__msg__PacketPass__rosidl_typesupport_introspection_c__PacketPass_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
