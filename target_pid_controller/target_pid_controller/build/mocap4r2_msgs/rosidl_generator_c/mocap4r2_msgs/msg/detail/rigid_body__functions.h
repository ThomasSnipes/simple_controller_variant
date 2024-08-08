// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mocap4r2_msgs:msg/RigidBody.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODY__FUNCTIONS_H_
#define MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mocap4r2_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mocap4r2_msgs/msg/detail/rigid_body__struct.h"

/// Initialize msg/RigidBody message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mocap4r2_msgs__msg__RigidBody
 * )) before or use
 * mocap4r2_msgs__msg__RigidBody__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
bool
mocap4r2_msgs__msg__RigidBody__init(mocap4r2_msgs__msg__RigidBody * msg);

/// Finalize msg/RigidBody message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
void
mocap4r2_msgs__msg__RigidBody__fini(mocap4r2_msgs__msg__RigidBody * msg);

/// Create msg/RigidBody message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mocap4r2_msgs__msg__RigidBody__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
mocap4r2_msgs__msg__RigidBody *
mocap4r2_msgs__msg__RigidBody__create();

/// Destroy msg/RigidBody message.
/**
 * It calls
 * mocap4r2_msgs__msg__RigidBody__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
void
mocap4r2_msgs__msg__RigidBody__destroy(mocap4r2_msgs__msg__RigidBody * msg);

/// Check for msg/RigidBody message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
bool
mocap4r2_msgs__msg__RigidBody__are_equal(const mocap4r2_msgs__msg__RigidBody * lhs, const mocap4r2_msgs__msg__RigidBody * rhs);

/// Copy a msg/RigidBody message.
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
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
bool
mocap4r2_msgs__msg__RigidBody__copy(
  const mocap4r2_msgs__msg__RigidBody * input,
  mocap4r2_msgs__msg__RigidBody * output);

/// Initialize array of msg/RigidBody messages.
/**
 * It allocates the memory for the number of elements and calls
 * mocap4r2_msgs__msg__RigidBody__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
bool
mocap4r2_msgs__msg__RigidBody__Sequence__init(mocap4r2_msgs__msg__RigidBody__Sequence * array, size_t size);

/// Finalize array of msg/RigidBody messages.
/**
 * It calls
 * mocap4r2_msgs__msg__RigidBody__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
void
mocap4r2_msgs__msg__RigidBody__Sequence__fini(mocap4r2_msgs__msg__RigidBody__Sequence * array);

/// Create array of msg/RigidBody messages.
/**
 * It allocates the memory for the array and calls
 * mocap4r2_msgs__msg__RigidBody__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
mocap4r2_msgs__msg__RigidBody__Sequence *
mocap4r2_msgs__msg__RigidBody__Sequence__create(size_t size);

/// Destroy array of msg/RigidBody messages.
/**
 * It calls
 * mocap4r2_msgs__msg__RigidBody__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
void
mocap4r2_msgs__msg__RigidBody__Sequence__destroy(mocap4r2_msgs__msg__RigidBody__Sequence * array);

/// Check for msg/RigidBody message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
bool
mocap4r2_msgs__msg__RigidBody__Sequence__are_equal(const mocap4r2_msgs__msg__RigidBody__Sequence * lhs, const mocap4r2_msgs__msg__RigidBody__Sequence * rhs);

/// Copy an array of msg/RigidBody messages.
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
ROSIDL_GENERATOR_C_PUBLIC_mocap4r2_msgs
bool
mocap4r2_msgs__msg__RigidBody__Sequence__copy(
  const mocap4r2_msgs__msg__RigidBody__Sequence * input,
  mocap4r2_msgs__msg__RigidBody__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODY__FUNCTIONS_H_
