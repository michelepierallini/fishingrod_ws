// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pi3hat_moteus_int_msgs:msg/JointsCommand.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__STRUCT_H_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__STRUCT_H_

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
// Member 'kp_scale'
// Member 'kd_scale'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/JointsCommand in the package pi3hat_moteus_int_msgs.
typedef struct pi3hat_moteus_int_msgs__msg__JointsCommand
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__double__Sequence position;
  rosidl_runtime_c__double__Sequence velocity;
  rosidl_runtime_c__double__Sequence effort;
  rosidl_runtime_c__double__Sequence kp_scale;
  rosidl_runtime_c__double__Sequence kd_scale;
} pi3hat_moteus_int_msgs__msg__JointsCommand;

// Struct for a sequence of pi3hat_moteus_int_msgs__msg__JointsCommand.
typedef struct pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence
{
  pi3hat_moteus_int_msgs__msg__JointsCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pi3hat_moteus_int_msgs__msg__JointsCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__STRUCT_H_
