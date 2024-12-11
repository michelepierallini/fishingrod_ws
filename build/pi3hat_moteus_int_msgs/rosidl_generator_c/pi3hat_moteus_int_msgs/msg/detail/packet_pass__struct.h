// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pi3hat_moteus_int_msgs:msg/PacketPass.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__STRUCT_H_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__STRUCT_H_

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
// Member 'pack_loss'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/PacketPass in the package pi3hat_moteus_int_msgs.
typedef struct pi3hat_moteus_int_msgs__msg__PacketPass
{
  std_msgs__msg__Header header;
  /// percentage of valid package recived
  double valid;
  /// the duration of the cycle operation in seconds
  double cycle_dur;
  /// motors id
  rosidl_runtime_c__String__Sequence name;
  /// percentage of corrected recived packet identify by motor id
  rosidl_runtime_c__double__Sequence pack_loss;
} pi3hat_moteus_int_msgs__msg__PacketPass;

// Struct for a sequence of pi3hat_moteus_int_msgs__msg__PacketPass.
typedef struct pi3hat_moteus_int_msgs__msg__PacketPass__Sequence
{
  pi3hat_moteus_int_msgs__msg__PacketPass * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pi3hat_moteus_int_msgs__msg__PacketPass__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__STRUCT_H_
