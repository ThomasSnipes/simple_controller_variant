// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mocap4r2_msgs:srv/CreateRigidBody.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mocap4r2_msgs/srv/detail/create_rigid_body__rosidl_typesupport_introspection_c.h"
#include "mocap4r2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mocap4r2_msgs/srv/detail/create_rigid_body__functions.h"
#include "mocap4r2_msgs/srv/detail/create_rigid_body__struct.h"


// Include directives for member types
// Member `rigid_body_name`
// Member `link_parent`
#include "rosidl_runtime_c/string_functions.h"
// Member `markers`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mocap4r2_msgs__srv__CreateRigidBody_Request__init(message_memory);
}

void mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_fini_function(void * message_memory)
{
  mocap4r2_msgs__srv__CreateRigidBody_Request__fini(message_memory);
}

size_t mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__size_function__CreateRigidBody_Request__markers(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__get_const_function__CreateRigidBody_Request__markers(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__get_function__CreateRigidBody_Request__markers(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__fetch_function__CreateRigidBody_Request__markers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__get_const_function__CreateRigidBody_Request__markers(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__assign_function__CreateRigidBody_Request__markers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__get_function__CreateRigidBody_Request__markers(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__resize_function__CreateRigidBody_Request__markers(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_message_member_array[3] = {
  {
    "rigid_body_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_msgs__srv__CreateRigidBody_Request, rigid_body_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "link_parent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_msgs__srv__CreateRigidBody_Request, link_parent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "markers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_msgs__srv__CreateRigidBody_Request, markers),  // bytes offset in struct
    NULL,  // default value
    mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__size_function__CreateRigidBody_Request__markers,  // size() function pointer
    mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__get_const_function__CreateRigidBody_Request__markers,  // get_const(index) function pointer
    mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__get_function__CreateRigidBody_Request__markers,  // get(index) function pointer
    mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__fetch_function__CreateRigidBody_Request__markers,  // fetch(index, &value) function pointer
    mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__assign_function__CreateRigidBody_Request__markers,  // assign(index, value) function pointer
    mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__resize_function__CreateRigidBody_Request__markers  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_message_members = {
  "mocap4r2_msgs__srv",  // message namespace
  "CreateRigidBody_Request",  // message name
  3,  // number of fields
  sizeof(mocap4r2_msgs__srv__CreateRigidBody_Request),
  mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_message_member_array,  // message members
  mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_message_type_support_handle = {
  0,
  &mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mocap4r2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mocap4r2_msgs, srv, CreateRigidBody_Request)() {
  if (!mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_message_type_support_handle.typesupport_identifier) {
    mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mocap4r2_msgs__srv__CreateRigidBody_Request__rosidl_typesupport_introspection_c__CreateRigidBody_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mocap4r2_msgs/srv/detail/create_rigid_body__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mocap4r2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mocap4r2_msgs/srv/detail/create_rigid_body__functions.h"
// already included above
// #include "mocap4r2_msgs/srv/detail/create_rigid_body__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mocap4r2_msgs__srv__CreateRigidBody_Response__init(message_memory);
}

void mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_fini_function(void * message_memory)
{
  mocap4r2_msgs__srv__CreateRigidBody_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mocap4r2_msgs__srv__CreateRigidBody_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_message_members = {
  "mocap4r2_msgs__srv",  // message namespace
  "CreateRigidBody_Response",  // message name
  1,  // number of fields
  sizeof(mocap4r2_msgs__srv__CreateRigidBody_Response),
  mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_message_member_array,  // message members
  mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_message_type_support_handle = {
  0,
  &mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mocap4r2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mocap4r2_msgs, srv, CreateRigidBody_Response)() {
  if (!mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_message_type_support_handle.typesupport_identifier) {
    mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mocap4r2_msgs__srv__CreateRigidBody_Response__rosidl_typesupport_introspection_c__CreateRigidBody_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mocap4r2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mocap4r2_msgs/srv/detail/create_rigid_body__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mocap4r2_msgs__srv__detail__create_rigid_body__rosidl_typesupport_introspection_c__CreateRigidBody_service_members = {
  "mocap4r2_msgs__srv",  // service namespace
  "CreateRigidBody",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // mocap4r2_msgs__srv__detail__create_rigid_body__rosidl_typesupport_introspection_c__CreateRigidBody_Request_message_type_support_handle,
  NULL  // response message
  // mocap4r2_msgs__srv__detail__create_rigid_body__rosidl_typesupport_introspection_c__CreateRigidBody_Response_message_type_support_handle
};

static rosidl_service_type_support_t mocap4r2_msgs__srv__detail__create_rigid_body__rosidl_typesupport_introspection_c__CreateRigidBody_service_type_support_handle = {
  0,
  &mocap4r2_msgs__srv__detail__create_rigid_body__rosidl_typesupport_introspection_c__CreateRigidBody_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mocap4r2_msgs, srv, CreateRigidBody_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mocap4r2_msgs, srv, CreateRigidBody_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mocap4r2_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mocap4r2_msgs, srv, CreateRigidBody)() {
  if (!mocap4r2_msgs__srv__detail__create_rigid_body__rosidl_typesupport_introspection_c__CreateRigidBody_service_type_support_handle.typesupport_identifier) {
    mocap4r2_msgs__srv__detail__create_rigid_body__rosidl_typesupport_introspection_c__CreateRigidBody_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mocap4r2_msgs__srv__detail__create_rigid_body__rosidl_typesupport_introspection_c__CreateRigidBody_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mocap4r2_msgs, srv, CreateRigidBody_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mocap4r2_msgs, srv, CreateRigidBody_Response)()->data;
  }

  return &mocap4r2_msgs__srv__detail__create_rigid_body__rosidl_typesupport_introspection_c__CreateRigidBody_service_type_support_handle;
}
