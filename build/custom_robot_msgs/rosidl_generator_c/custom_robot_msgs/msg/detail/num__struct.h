// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_robot_msgs:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_ROBOT_MSGS__MSG__DETAIL__NUM__STRUCT_H_
#define CUSTOM_ROBOT_MSGS__MSG__DETAIL__NUM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Num in the package custom_robot_msgs.
typedef struct custom_robot_msgs__msg__Num
{
  int64_t num;
} custom_robot_msgs__msg__Num;

// Struct for a sequence of custom_robot_msgs__msg__Num.
typedef struct custom_robot_msgs__msg__Num__Sequence
{
  custom_robot_msgs__msg__Num * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_robot_msgs__msg__Num__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_ROBOT_MSGS__MSG__DETAIL__NUM__STRUCT_H_
