// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mocap4r2_msgs:msg/RigidBodies.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODIES__BUILDER_HPP_
#define MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODIES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mocap4r2_msgs/msg/detail/rigid_bodies__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mocap4r2_msgs
{

namespace msg
{

namespace builder
{

class Init_RigidBodies_rigidbodies
{
public:
  explicit Init_RigidBodies_rigidbodies(::mocap4r2_msgs::msg::RigidBodies & msg)
  : msg_(msg)
  {}
  ::mocap4r2_msgs::msg::RigidBodies rigidbodies(::mocap4r2_msgs::msg::RigidBodies::_rigidbodies_type arg)
  {
    msg_.rigidbodies = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mocap4r2_msgs::msg::RigidBodies msg_;
};

class Init_RigidBodies_frame_number
{
public:
  explicit Init_RigidBodies_frame_number(::mocap4r2_msgs::msg::RigidBodies & msg)
  : msg_(msg)
  {}
  Init_RigidBodies_rigidbodies frame_number(::mocap4r2_msgs::msg::RigidBodies::_frame_number_type arg)
  {
    msg_.frame_number = std::move(arg);
    return Init_RigidBodies_rigidbodies(msg_);
  }

private:
  ::mocap4r2_msgs::msg::RigidBodies msg_;
};

class Init_RigidBodies_header
{
public:
  Init_RigidBodies_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RigidBodies_frame_number header(::mocap4r2_msgs::msg::RigidBodies::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RigidBodies_frame_number(msg_);
  }

private:
  ::mocap4r2_msgs::msg::RigidBodies msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mocap4r2_msgs::msg::RigidBodies>()
{
  return mocap4r2_msgs::msg::builder::Init_RigidBodies_header();
}

}  // namespace mocap4r2_msgs

#endif  // MOCAP4R2_MSGS__MSG__DETAIL__RIGID_BODIES__BUILDER_HPP_
