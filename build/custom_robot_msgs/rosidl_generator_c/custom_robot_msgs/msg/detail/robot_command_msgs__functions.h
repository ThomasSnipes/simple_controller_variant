// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from custom_robot_msgs:msg/RobotCommandMsgs.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__FUNCTIONS_H_
#define CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "custom_robot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "custom_robot_msgs/msg/detail/robot_command_msgs__struct.h"

/// Initialize msg/RobotCommandMsgs message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * custom_robot_msgs__msg__RobotCommandMsgs
 * )) before or use
 * custom_robot_msgs__msg__RobotCommandMsgs__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
bool
custom_robot_msgs__msg__RobotCommandMsgs__init(custom_robot_msgs__msg__RobotCommandMsgs * msg);

/// Finalize msg/RobotCommandMsgs message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
void
custom_robot_msgs__msg__RobotCommandMsgs__fini(custom_robot_msgs__msg__RobotCommandMsgs * msg);

/// Create msg/RobotCommandMsgs message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * custom_robot_msgs__msg__RobotCommandMsgs__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
custom_robot_msgs__msg__RobotCommandMsgs *
custom_robot_msgs__msg__RobotCommandMsgs__create();

/// Destroy msg/RobotCommandMsgs message.
/**
 * It calls
 * custom_robot_msgs__msg__RobotCommandMsgs__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
void
custom_robot_msgs__msg__RobotCommandMsgs__destroy(custom_robot_msgs__msg__RobotCommandMsgs * msg);

/// Check for msg/RobotCommandMsgs message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
bool
custom_robot_msgs__msg__RobotCommandMsgs__are_equal(const custom_robot_msgs__msg__RobotCommandMsgs * lhs, const custom_robot_msgs__msg__RobotCommandMsgs * rhs);

/// Copy a msg/RobotCommandMsgs message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
bool
custom_robot_msgs__msg__RobotCommandMsgs__copy(
  const custom_robot_msgs__msg__RobotCommandMsgs * input,
  custom_robot_msgs__msg__RobotCommandMsgs * output);

/// Initialize array of msg/RobotCommandMsgs messages.
/**
 * It allocates the memory for the number of elements and calls
 * custom_robot_msgs__msg__RobotCommandMsgs__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
bool
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__init(custom_robot_msgs__msg__RobotCommandMsgs__Sequence * array, size_t size);

/// Finalize array of msg/RobotCommandMsgs messages.
/**
 * It calls
 * custom_robot_msgs__msg__RobotCommandMsgs__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
void
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__fini(custom_robot_msgs__msg__RobotCommandMsgs__Sequence * array);

/// Create array of msg/RobotCommandMsgs messages.
/**
 * It allocates the memory for the array and calls
 * custom_robot_msgs__msg__RobotCommandMsgs__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
custom_robot_msgs__msg__RobotCommandMsgs__Sequence *
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__create(size_t size);

/// Destroy array of msg/RobotCommandMsgs messages.
/**
 * It calls
 * custom_robot_msgs__msg__RobotCommandMsgs__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
void
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__destroy(custom_robot_msgs__msg__RobotCommandMsgs__Sequence * array);

/// Check for msg/RobotCommandMsgs message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
bool
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__are_equal(const custom_robot_msgs__msg__RobotCommandMsgs__Sequence * lhs, const custom_robot_msgs__msg__RobotCommandMsgs__Sequence * rhs);

/// Copy an array of msg/RobotCommandMsgs messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_robot_msgs
bool
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__copy(
  const custom_robot_msgs__msg__RobotCommandMsgs__Sequence * input,
  custom_robot_msgs__msg__RobotCommandMsgs__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__FUNCTIONS_H_
