// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_robot_msgs:msg/Num.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_ROBOT_MSGS__MSG__DETAIL__NUM__BUILDER_HPP_
#define CUSTOM_ROBOT_MSGS__MSG__DETAIL__NUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_robot_msgs/msg/detail/num__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_robot_msgs
{

namespace msg
{

namespace builder
{

class Init_Num_num
{
public:
  Init_Num_num()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_robot_msgs::msg::Num num(::custom_robot_msgs::msg::Num::_num_type arg)
  {
    msg_.num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_robot_msgs::msg::Num msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_robot_msgs::msg::Num>()
{
  return custom_robot_msgs::msg::builder::Init_Num_num();
}

}  // namespace custom_robot_msgs

#endif  // CUSTOM_ROBOT_MSGS__MSG__DETAIL__NUM__BUILDER_HPP_
