// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mocap4r2_msgs:srv/CreateRigidBody.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__STRUCT_H_
#define MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'rigid_body_name'
// Member 'link_parent'
#include "rosidl_runtime_c/string.h"
// Member 'markers'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/CreateRigidBody in the package mocap4r2_msgs.
typedef struct mocap4r2_msgs__srv__CreateRigidBody_Request
{
  rosidl_runtime_c__String rigid_body_name;
  rosidl_runtime_c__String link_parent;
  rosidl_runtime_c__int32__Sequence markers;
} mocap4r2_msgs__srv__CreateRigidBody_Request;

// Struct for a sequence of mocap4r2_msgs__srv__CreateRigidBody_Request.
typedef struct mocap4r2_msgs__srv__CreateRigidBody_Request__Sequence
{
  mocap4r2_msgs__srv__CreateRigidBody_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mocap4r2_msgs__srv__CreateRigidBody_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CreateRigidBody in the package mocap4r2_msgs.
typedef struct mocap4r2_msgs__srv__CreateRigidBody_Response
{
  bool success;
} mocap4r2_msgs__srv__CreateRigidBody_Response;

// Struct for a sequence of mocap4r2_msgs__srv__CreateRigidBody_Response.
typedef struct mocap4r2_msgs__srv__CreateRigidBody_Response__Sequence
{
  mocap4r2_msgs__srv__CreateRigidBody_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mocap4r2_msgs__srv__CreateRigidBody_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__STRUCT_H_
