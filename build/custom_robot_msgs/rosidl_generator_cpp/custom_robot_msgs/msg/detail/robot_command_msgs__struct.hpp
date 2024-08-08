// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_robot_msgs:msg/RobotCommandMsgs.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__STRUCT_HPP_
#define CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_robot_msgs__msg__RobotCommandMsgs __attribute__((deprecated))
#else
# define DEPRECATED__custom_robot_msgs__msg__RobotCommandMsgs __declspec(deprecated)
#endif

namespace custom_robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotCommandMsgs_
{
  using Type = RobotCommandMsgs_<ContainerAllocator>;

  explicit RobotCommandMsgs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->x_target = 0.0f;
      this->y_target = 0.0f;
      this->orientation = 0.0f;
      this->angle = 0.0f;
    }
  }

  explicit RobotCommandMsgs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
      this->x = 0.0f;
      this->y = 0.0f;
      this->x_target = 0.0f;
      this->y_target = 0.0f;
      this->orientation = 0.0f;
      this->angle = 0.0f;
    }
  }

  // field types and members
  using _linear_velocity_type =
    float;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    float;
  _angular_velocity_type angular_velocity;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _x_target_type =
    float;
  _x_target_type x_target;
  using _y_target_type =
    float;
  _y_target_type y_target;
  using _orientation_type =
    float;
  _orientation_type orientation;
  using _angle_type =
    float;
  _angle_type angle;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;

  // setters for named parameter idiom
  Type & set__linear_velocity(
    const float & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const float & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__x_target(
    const float & _arg)
  {
    this->x_target = _arg;
    return *this;
  }
  Type & set__y_target(
    const float & _arg)
  {
    this->y_target = _arg;
    return *this;
  }
  Type & set__orientation(
    const float & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_robot_msgs__msg__RobotCommandMsgs
    std::shared_ptr<custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_robot_msgs__msg__RobotCommandMsgs
    std::shared_ptr<custom_robot_msgs::msg::RobotCommandMsgs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotCommandMsgs_ & other) const
  {
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->x_target != other.x_target) {
      return false;
    }
    if (this->y_target != other.y_target) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotCommandMsgs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotCommandMsgs_

// alias to use template instance with default allocator
using RobotCommandMsgs =
  custom_robot_msgs::msg::RobotCommandMsgs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_robot_msgs

#endif  // CUSTOM_ROBOT_MSGS__MSG__DETAIL__ROBOT_COMMAND_MSGS__STRUCT_HPP_
