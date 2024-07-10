// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_robot_msgs:msg/RobotCommandMsgs.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__STRUCT_H_
#define CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/RobotCommandMsgs in the package custom_robot_msgs.
typedef struct custom_robot_msgs__msg__RobotCommandMsgs
{
  /// Linear velocity command
  float linear_velocity;
  /// Angular velocity command
  float angular_velocity;
  geometry_msgs__msg__Point position;
} custom_robot_msgs__msg__RobotCommandMsgs;

// Struct for a sequence of custom_robot_msgs__msg__RobotCommandMsgs.
typedef struct custom_robot_msgs__msg__RobotCommandMsgs__Sequence
{
  custom_robot_msgs__msg__RobotCommandMsgs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_robot_msgs__msg__RobotCommandMsgs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__STRUCT_H_
