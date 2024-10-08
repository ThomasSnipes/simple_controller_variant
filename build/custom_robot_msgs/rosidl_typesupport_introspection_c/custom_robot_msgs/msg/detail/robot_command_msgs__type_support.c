// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_robot_msgs:msg/RobotCommandMsgs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_robot_msgs/msg/detail/robot_command_msgs__rosidl_typesupport_introspection_c.h"
#include "custom_robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_robot_msgs/msg/detail/robot_command_msgs__functions.h"
#include "custom_robot_msgs/msg/detail/robot_command_msgs__struct.h"


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_robot_msgs__msg__RobotCommandMsgs__init(message_memory);
}

void custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_fini_function(void * message_memory)
{
  custom_robot_msgs__msg__RobotCommandMsgs__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_message_member_array[9] = {
  {
    "linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_robot_msgs__msg__RobotCommandMsgs, linear_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_robot_msgs__msg__RobotCommandMsgs, angular_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_robot_msgs__msg__RobotCommandMsgs, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_robot_msgs__msg__RobotCommandMsgs, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x_target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_robot_msgs__msg__RobotCommandMsgs, x_target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y_target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_robot_msgs__msg__RobotCommandMsgs, y_target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_robot_msgs__msg__RobotCommandMsgs, orientation),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_robot_msgs__msg__RobotCommandMsgs, angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_robot_msgs__msg__RobotCommandMsgs, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_message_members = {
  "custom_robot_msgs__msg",  // message namespace
  "RobotCommandMsgs",  // message name
  9,  // number of fields
  sizeof(custom_robot_msgs__msg__RobotCommandMsgs),
  custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_message_member_array,  // message members
  custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_message_type_support_handle = {
  0,
  &custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_robot_msgs, msg, RobotCommandMsgs)() {
  custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_message_type_support_handle.typesupport_identifier) {
    custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_robot_msgs__msg__RobotCommandMsgs__rosidl_typesupport_introspection_c__RobotCommandMsgs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
