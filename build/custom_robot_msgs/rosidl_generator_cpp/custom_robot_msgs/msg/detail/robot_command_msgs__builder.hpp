// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_robot_msgs:msg/RobotCommandMsgs.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__BUILDER_HPP_
#define CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_robot_msgs/msg/detail/robot_command_msgs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_robot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotCommandMsgs_position
{
public:
  explicit Init_RobotCommandMsgs_position(::custom_robot_msgs::msg::RobotCommandMsgs & msg)
  : msg_(msg)
  {}
  ::custom_robot_msgs::msg::RobotCommandMsgs position(::custom_robot_msgs::msg::RobotCommandMsgs::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_robot_msgs::msg::RobotCommandMsgs msg_;
};

class Init_RobotCommandMsgs_angle
{
public:
  explicit Init_RobotCommandMsgs_angle(::custom_robot_msgs::msg::RobotCommandMsgs & msg)
  : msg_(msg)
  {}
  Init_RobotCommandMsgs_position angle(::custom_robot_msgs::msg::RobotCommandMsgs::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_RobotCommandMsgs_position(msg_);
  }

private:
  ::custom_robot_msgs::msg::RobotCommandMsgs msg_;
};

class Init_RobotCommandMsgs_orientation
{
public:
  explicit Init_RobotCommandMsgs_orientation(::custom_robot_msgs::msg::RobotCommandMsgs & msg)
  : msg_(msg)
  {}
  Init_RobotCommandMsgs_angle orientation(::custom_robot_msgs::msg::RobotCommandMsgs::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_RobotCommandMsgs_angle(msg_);
  }

private:
  ::custom_robot_msgs::msg::RobotCommandMsgs msg_;
};

class Init_RobotCommandMsgs_y_target
{
public:
  explicit Init_RobotCommandMsgs_y_target(::custom_robot_msgs::msg::RobotCommandMsgs & msg)
  : msg_(msg)
  {}
  Init_RobotCommandMsgs_orientation y_target(::custom_robot_msgs::msg::RobotCommandMsgs::_y_target_type arg)
  {
    msg_.y_target = std::move(arg);
    return Init_RobotCommandMsgs_orientation(msg_);
  }

private:
  ::custom_robot_msgs::msg::RobotCommandMsgs msg_;
};

class Init_RobotCommandMsgs_x_target
{
public:
  explicit Init_RobotCommandMsgs_x_target(::custom_robot_msgs::msg::RobotCommandMsgs & msg)
  : msg_(msg)
  {}
  Init_RobotCommandMsgs_y_target x_target(::custom_robot_msgs::msg::RobotCommandMsgs::_x_target_type arg)
  {
    msg_.x_target = std::move(arg);
    return Init_RobotCommandMsgs_y_target(msg_);
  }

private:
  ::custom_robot_msgs::msg::RobotCommandMsgs msg_;
};

class Init_RobotCommandMsgs_y
{
public:
  explicit Init_RobotCommandMsgs_y(::custom_robot_msgs::msg::RobotCommandMsgs & msg)
  : msg_(msg)
  {}
  Init_RobotCommandMsgs_x_target y(::custom_robot_msgs::msg::RobotCommandMsgs::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_RobotCommandMsgs_x_target(msg_);
  }

private:
  ::custom_robot_msgs::msg::RobotCommandMsgs msg_;
};

class Init_RobotCommandMsgs_x
{
public:
  explicit Init_RobotCommandMsgs_x(::custom_robot_msgs::msg::RobotCommandMsgs & msg)
  : msg_(msg)
  {}
  Init_RobotCommandMsgs_y x(::custom_robot_msgs::msg::RobotCommandMsgs::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RobotCommandMsgs_y(msg_);
  }

private:
  ::custom_robot_msgs::msg::RobotCommandMsgs msg_;
};

class Init_RobotCommandMsgs_angular_velocity
{
public:
  explicit Init_RobotCommandMsgs_angular_velocity(::custom_robot_msgs::msg::RobotCommandMsgs & msg)
  : msg_(msg)
  {}
  Init_RobotCommandMsgs_x angular_velocity(::custom_robot_msgs::msg::RobotCommandMsgs::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_RobotCommandMsgs_x(msg_);
  }

private:
  ::custom_robot_msgs::msg::RobotCommandMsgs msg_;
};

class Init_RobotCommandMsgs_linear_velocity
{
public:
  Init_RobotCommandMsgs_linear_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotCommandMsgs_angular_velocity linear_velocity(::custom_robot_msgs::msg::RobotCommandMsgs::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_RobotCommandMsgs_angular_velocity(msg_);
  }

private:
  ::custom_robot_msgs::msg::RobotCommandMsgs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_robot_msgs::msg::RobotCommandMsgs>()
{
  return custom_robot_msgs::msg::builder::Init_RobotCommandMsgs_linear_velocity();
}

}  // namespace custom_robot_msgs

#endif  // CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__BUILDER_HPP_
