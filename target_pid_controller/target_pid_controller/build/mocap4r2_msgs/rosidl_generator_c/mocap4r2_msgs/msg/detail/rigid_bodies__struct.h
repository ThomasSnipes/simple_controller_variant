// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mocap4r2_msgs:msg/RigidBodies.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODIES__STRUCT_H_
#define MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODIES__STRUCT_H_

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
// Member 'rigidbodies'
#include "mocap4r2_msgs/msg/detail/rigid_body__struct.h"

/// Struct defined in msg/RigidBodies in the package mocap4r2_msgs.
typedef struct mocap4r2_msgs__msg__RigidBodies
{
  std_msgs__msg__Header header;
  uint32_t frame_number;
  mocap4r2_msgs__msg__RigidBody__Sequence rigidbodies;
} mocap4r2_msgs__msg__RigidBodies;

// Struct for a sequence of mocap4r2_msgs__msg__RigidBodies.
typedef struct mocap4r2_msgs__msg__RigidBodies__Sequence
{
  mocap4r2_msgs__msg__RigidBodies * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mocap4r2_msgs__msg__RigidBodies__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODIES__STRUCT_H_
