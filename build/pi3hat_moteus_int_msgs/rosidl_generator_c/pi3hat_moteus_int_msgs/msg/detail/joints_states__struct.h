// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pi3hat_moteus_int_msgs:msg/JointsStates.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_STATES__STRUCT_H_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'position'
// Member 'velocity'
// Member 'effort'
// Member 'current'
// Member 'temperature'
// Member 'sec_enc_pos'
// Member 'sec_enc_vel'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/JointsStates in the package pi3hat_moteus_int_msgs.
typedef struct pi3hat_moteus_int_msgs__msg__JointsStates
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__double__Sequence position;
  rosidl_runtime_c__double__Sequence velocity;
  rosidl_runtime_c__double__Sequence effort;
  rosidl_runtime_c__double__Sequence current;
  rosidl_runtime_c__double__Sequence temperature;
  rosidl_runtime_c__double__Sequence sec_enc_pos;
  rosidl_runtime_c__double__Sequence sec_enc_vel;
} pi3hat_moteus_int_msgs__msg__JointsStates;

// Struct for a sequence of pi3hat_moteus_int_msgs__msg__JointsStates.
typedef struct pi3hat_moteus_int_msgs__msg__JointsStates__Sequence
{
  pi3hat_moteus_int_msgs__msg__JointsStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pi3hat_moteus_int_msgs__msg__JointsStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_STATES__STRUCT_H_
