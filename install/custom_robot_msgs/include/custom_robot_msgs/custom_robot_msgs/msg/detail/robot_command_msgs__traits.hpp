// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_robot_msgs:msg/RobotCommandMsgs.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__TRAITS_HPP_
#define CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_robot_msgs/msg/detail/robot_command_msgs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace custom_robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotCommandMsgs & msg,
  std::ostream & out)
{
  out << "{";
  // member: linear_velocity
  {
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: x_target
  {
    out << "x_target: ";
    rosidl_generator_traits::value_to_yaml(msg.x_target, out);
    out << ", ";
  }

  // member: y_target
  {
    out << "y_target: ";
    rosidl_generator_traits::value_to_yaml(msg.y_target, out);
    out << ", ";
  }

  // member: orientation
  {
    out << "orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.orientation, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotCommandMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: linear_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << "\n";
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: x_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_target: ";
    rosidl_generator_traits::value_to_yaml(msg.x_target, out);
    out << "\n";
  }

  // member: y_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_target: ";
    rosidl_generator_traits::value_to_yaml(msg.y_target, out);
    out << "\n";
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.orientation, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotCommandMsgs & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_robot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_robot_msgs::msg::RobotCommandMsgs & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_robot_msgs::msg::RobotCommandMsgs & msg)
{
  return custom_robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_robot_msgs::msg::RobotCommandMsgs>()
{
  return "custom_robot_msgs::msg::RobotCommandMsgs";
}

template<>
inline const char * name<custom_robot_msgs::msg::RobotCommandMsgs>()
{
  return "custom_robot_msgs/msg/RobotCommandMsgs";
}

template<>
struct has_fixed_size<custom_robot_msgs::msg::RobotCommandMsgs>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<custom_robot_msgs::msg::RobotCommandMsgs>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<custom_robot_msgs::msg::RobotCommandMsgs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__TRAITS_HPP_
