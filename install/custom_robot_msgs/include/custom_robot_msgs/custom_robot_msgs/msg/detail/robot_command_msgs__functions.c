// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_robot_msgs:msg/RobotCommandMsgs.idl
// generated code does not contain a copyright notice
#include "custom_robot_msgs/msg/detail/robot_command_msgs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
custom_robot_msgs__msg__RobotCommandMsgs__init(custom_robot_msgs__msg__RobotCommandMsgs * msg)
{
  if (!msg) {
    return false;
  }
  // linear_velocity
  // angular_velocity
  // x
  // y
  // x_target
  // y_target
  // orientation
  // angle
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    custom_robot_msgs__msg__RobotCommandMsgs__fini(msg);
    return false;
  }
  return true;
}

void
custom_robot_msgs__msg__RobotCommandMsgs__fini(custom_robot_msgs__msg__RobotCommandMsgs * msg)
{
  if (!msg) {
    return;
  }
  // linear_velocity
  // angular_velocity
  // x
  // y
  // x_target
  // y_target
  // orientation
  // angle
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
}

bool
custom_robot_msgs__msg__RobotCommandMsgs__are_equal(const custom_robot_msgs__msg__RobotCommandMsgs * lhs, const custom_robot_msgs__msg__RobotCommandMsgs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // linear_velocity
  if (lhs->linear_velocity != rhs->linear_velocity) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // x_target
  if (lhs->x_target != rhs->x_target) {
    return false;
  }
  // y_target
  if (lhs->y_target != rhs->y_target) {
    return false;
  }
  // orientation
  if (lhs->orientation != rhs->orientation) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  return true;
}

bool
custom_robot_msgs__msg__RobotCommandMsgs__copy(
  const custom_robot_msgs__msg__RobotCommandMsgs * input,
  custom_robot_msgs__msg__RobotCommandMsgs * output)
{
  if (!input || !output) {
    return false;
  }
  // linear_velocity
  output->linear_velocity = input->linear_velocity;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // x_target
  output->x_target = input->x_target;
  // y_target
  output->y_target = input->y_target;
  // orientation
  output->orientation = input->orientation;
  // angle
  output->angle = input->angle;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  return true;
}

custom_robot_msgs__msg__RobotCommandMsgs *
custom_robot_msgs__msg__RobotCommandMsgs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_robot_msgs__msg__RobotCommandMsgs * msg = (custom_robot_msgs__msg__RobotCommandMsgs *)allocator.allocate(sizeof(custom_robot_msgs__msg__RobotCommandMsgs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_robot_msgs__msg__RobotCommandMsgs));
  bool success = custom_robot_msgs__msg__RobotCommandMsgs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_robot_msgs__msg__RobotCommandMsgs__destroy(custom_robot_msgs__msg__RobotCommandMsgs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_robot_msgs__msg__RobotCommandMsgs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__init(custom_robot_msgs__msg__RobotCommandMsgs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_robot_msgs__msg__RobotCommandMsgs * data = NULL;

  if (size) {
    data = (custom_robot_msgs__msg__RobotCommandMsgs *)allocator.zero_allocate(size, sizeof(custom_robot_msgs__msg__RobotCommandMsgs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_robot_msgs__msg__RobotCommandMsgs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_robot_msgs__msg__RobotCommandMsgs__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__fini(custom_robot_msgs__msg__RobotCommandMsgs__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_robot_msgs__msg__RobotCommandMsgs__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_robot_msgs__msg__RobotCommandMsgs__Sequence *
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_robot_msgs__msg__RobotCommandMsgs__Sequence * array = (custom_robot_msgs__msg__RobotCommandMsgs__Sequence *)allocator.allocate(sizeof(custom_robot_msgs__msg__RobotCommandMsgs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_robot_msgs__msg__RobotCommandMsgs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__destroy(custom_robot_msgs__msg__RobotCommandMsgs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_robot_msgs__msg__RobotCommandMsgs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__are_equal(const custom_robot_msgs__msg__RobotCommandMsgs__Sequence * lhs, const custom_robot_msgs__msg__RobotCommandMsgs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_robot_msgs__msg__RobotCommandMsgs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_robot_msgs__msg__RobotCommandMsgs__Sequence__copy(
  const custom_robot_msgs__msg__RobotCommandMsgs__Sequence * input,
  custom_robot_msgs__msg__RobotCommandMsgs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_robot_msgs__msg__RobotCommandMsgs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_robot_msgs__msg__RobotCommandMsgs * data =
      (custom_robot_msgs__msg__RobotCommandMsgs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_robot_msgs__msg__RobotCommandMsgs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_robot_msgs__msg__RobotCommandMsgs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_robot_msgs__msg__RobotCommandMsgs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
