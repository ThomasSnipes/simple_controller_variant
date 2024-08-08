// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mocap4r2_msgs:srv/CreateRigidBody.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__BUILDER_HPP_
#define MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mocap4r2_msgs/srv/detail/create_rigid_body__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mocap4r2_msgs
{

namespace srv
{

namespace builder
{

class Init_CreateRigidBody_Request_markers
{
public:
  explicit Init_CreateRigidBody_Request_markers(::mocap4r2_msgs::srv::CreateRigidBody_Request & msg)
  : msg_(msg)
  {}
  ::mocap4r2_msgs::srv::CreateRigidBody_Request markers(::mocap4r2_msgs::srv::CreateRigidBody_Request::_markers_type arg)
  {
    msg_.markers = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mocap4r2_msgs::srv::CreateRigidBody_Request msg_;
};

class Init_CreateRigidBody_Request_link_parent
{
public:
  explicit Init_CreateRigidBody_Request_link_parent(::mocap4r2_msgs::srv::CreateRigidBody_Request & msg)
  : msg_(msg)
  {}
  Init_CreateRigidBody_Request_markers link_parent(::mocap4r2_msgs::srv::CreateRigidBody_Request::_link_parent_type arg)
  {
    msg_.link_parent = std::move(arg);
    return Init_CreateRigidBody_Request_markers(msg_);
  }

private:
  ::mocap4r2_msgs::srv::CreateRigidBody_Request msg_;
};

class Init_CreateRigidBody_Request_rigid_body_name
{
public:
  Init_CreateRigidBody_Request_rigid_body_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CreateRigidBody_Request_link_parent rigid_body_name(::mocap4r2_msgs::srv::CreateRigidBody_Request::_rigid_body_name_type arg)
  {
    msg_.rigid_body_name = std::move(arg);
    return Init_CreateRigidBody_Request_link_parent(msg_);
  }

private:
  ::mocap4r2_msgs::srv::CreateRigidBody_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mocap4r2_msgs::srv::CreateRigidBody_Request>()
{
  return mocap4r2_msgs::srv::builder::Init_CreateRigidBody_Request_rigid_body_name();
}

}  // namespace mocap4r2_msgs


namespace mocap4r2_msgs
{

namespace srv
{

namespace builder
{

class Init_CreateRigidBody_Response_success
{
public:
  Init_CreateRigidBody_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mocap4r2_msgs::srv::CreateRigidBody_Response success(::mocap4r2_msgs::srv::CreateRigidBody_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mocap4r2_msgs::srv::CreateRigidBody_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mocap4r2_msgs::srv::CreateRigidBody_Response>()
{
  return mocap4r2_msgs::srv::builder::Init_CreateRigidBody_Response_success();
}

}  // namespace mocap4r2_msgs

#endif  // MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__BUILDER_HPP_
