// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mocap4r2_msgs:msg/RigidBodies.idl
// generated code does not contain a copyright notice
#include "mocap4r2_msgs/msg/detail/rigid_bodies__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `rigidbodies`
#include "mocap4r2_msgs/msg/detail/rigid_body__functions.h"

bool
mocap4r2_msgs__msg__RigidBodies__init(mocap4r2_msgs__msg__RigidBodies * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mocap4r2_msgs__msg__RigidBodies__fini(msg);
    return false;
  }
  // frame_number
  // rigidbodies
  if (!mocap4r2_msgs__msg__RigidBody__Sequence__init(&msg->rigidbodies, 0)) {
    mocap4r2_msgs__msg__RigidBodies__fini(msg);
    return false;
  }
  return true;
}

void
mocap4r2_msgs__msg__RigidBodies__fini(mocap4r2_msgs__msg__RigidBodies * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // frame_number
  // rigidbodies
  mocap4r2_msgs__msg__RigidBody__Sequence__fini(&msg->rigidbodies);
}

bool
mocap4r2_msgs__msg__RigidBodies__are_equal(const mocap4r2_msgs__msg__RigidBodies * lhs, const mocap4r2_msgs__msg__RigidBodies * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // frame_number
  if (lhs->frame_number != rhs->frame_number) {
    return false;
  }
  // rigidbodies
  if (!mocap4r2_msgs__msg__RigidBody__Sequence__are_equal(
      &(lhs->rigidbodies), &(rhs->rigidbodies)))
  {
    return false;
  }
  return true;
}

bool
mocap4r2_msgs__msg__RigidBodies__copy(
  const mocap4r2_msgs__msg__RigidBodies * input,
  mocap4r2_msgs__msg__RigidBodies * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // frame_number
  output->frame_number = input->frame_number;
  // rigidbodies
  if (!mocap4r2_msgs__msg__RigidBody__Sequence__copy(
      &(input->rigidbodies), &(output->rigidbodies)))
  {
    return false;
  }
  return true;
}

mocap4r2_msgs__msg__RigidBodies *
mocap4r2_msgs__msg__RigidBodies__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mocap4r2_msgs__msg__RigidBodies * msg = (mocap4r2_msgs__msg__RigidBodies *)allocator.allocate(sizeof(mocap4r2_msgs__msg__RigidBodies), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mocap4r2_msgs__msg__RigidBodies));
  bool success = mocap4r2_msgs__msg__RigidBodies__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mocap4r2_msgs__msg__RigidBodies__destroy(mocap4r2_msgs__msg__RigidBodies * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mocap4r2_msgs__msg__RigidBodies__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mocap4r2_msgs__msg__RigidBodies__Sequence__init(mocap4r2_msgs__msg__RigidBodies__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mocap4r2_msgs__msg__RigidBodies * data = NULL;

  if (size) {
    data = (mocap4r2_msgs__msg__RigidBodies *)allocator.zero_allocate(size, sizeof(mocap4r2_msgs__msg__RigidBodies), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mocap4r2_msgs__msg__RigidBodies__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mocap4r2_msgs__msg__RigidBodies__fini(&data[i - 1]);
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
mocap4r2_msgs__msg__RigidBodies__Sequence__fini(mocap4r2_msgs__msg__RigidBodies__Sequence * array)
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
      mocap4r2_msgs__msg__RigidBodies__fini(&array->data[i]);
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

mocap4r2_msgs__msg__RigidBodies__Sequence *
mocap4r2_msgs__msg__RigidBodies__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mocap4r2_msgs__msg__RigidBodies__Sequence * array = (mocap4r2_msgs__msg__RigidBodies__Sequence *)allocator.allocate(sizeof(mocap4r2_msgs__msg__RigidBodies__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mocap4r2_msgs__msg__RigidBodies__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mocap4r2_msgs__msg__RigidBodies__Sequence__destroy(mocap4r2_msgs__msg__RigidBodies__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mocap4r2_msgs__msg__RigidBodies__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mocap4r2_msgs__msg__RigidBodies__Sequence__are_equal(const mocap4r2_msgs__msg__RigidBodies__Sequence * lhs, const mocap4r2_msgs__msg__RigidBodies__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mocap4r2_msgs__msg__RigidBodies__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mocap4r2_msgs__msg__RigidBodies__Sequence__copy(
  const mocap4r2_msgs__msg__RigidBodies__Sequence * input,
  mocap4r2_msgs__msg__RigidBodies__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mocap4r2_msgs__msg__RigidBodies);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mocap4r2_msgs__msg__RigidBodies * data =
      (mocap4r2_msgs__msg__RigidBodies *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mocap4r2_msgs__msg__RigidBodies__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mocap4r2_msgs__msg__RigidBodies__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mocap4r2_msgs__msg__RigidBodies__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
