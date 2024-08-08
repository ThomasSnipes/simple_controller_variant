// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mocap4r2_msgs:srv/CreateRigidBody.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__TRAITS_HPP_
#define MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mocap4r2_msgs/srv/detail/create_rigid_body__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mocap4r2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CreateRigidBody_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: rigid_body_name
  {
    out << "rigid_body_name: ";
    rosidl_generator_traits::value_to_yaml(msg.rigid_body_name, out);
    out << ", ";
  }

  // member: link_parent
  {
    out << "link_parent: ";
    rosidl_generator_traits::value_to_yaml(msg.link_parent, out);
    out << ", ";
  }

  // member: markers
  {
    if (msg.markers.size() == 0) {
      out << "markers: []";
    } else {
      out << "markers: [";
      size_t pending_items = msg.markers.size();
      for (auto item : msg.markers) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CreateRigidBody_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rigid_body_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rigid_body_name: ";
    rosidl_generator_traits::value_to_yaml(msg.rigid_body_name, out);
    out << "\n";
  }

  // member: link_parent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "link_parent: ";
    rosidl_generator_traits::value_to_yaml(msg.link_parent, out);
    out << "\n";
  }

  // member: markers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.markers.size() == 0) {
      out << "markers: []\n";
    } else {
      out << "markers:\n";
      for (auto item : msg.markers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CreateRigidBody_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mocap4r2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mocap4r2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mocap4r2_msgs::srv::CreateRigidBody_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mocap4r2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mocap4r2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mocap4r2_msgs::srv::CreateRigidBody_Request & msg)
{
  return mocap4r2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mocap4r2_msgs::srv::CreateRigidBody_Request>()
{
  return "mocap4r2_msgs::srv::CreateRigidBody_Request";
}

template<>
inline const char * name<mocap4r2_msgs::srv::CreateRigidBody_Request>()
{
  return "mocap4r2_msgs/srv/CreateRigidBody_Request";
}

template<>
struct has_fixed_size<mocap4r2_msgs::srv::CreateRigidBody_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mocap4r2_msgs::srv::CreateRigidBody_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mocap4r2_msgs::srv::CreateRigidBody_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mocap4r2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CreateRigidBody_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CreateRigidBody_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CreateRigidBody_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mocap4r2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mocap4r2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mocap4r2_msgs::srv::CreateRigidBody_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mocap4r2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mocap4r2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mocap4r2_msgs::srv::CreateRigidBody_Response & msg)
{
  return mocap4r2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mocap4r2_msgs::srv::CreateRigidBody_Response>()
{
  return "mocap4r2_msgs::srv::CreateRigidBody_Response";
}

template<>
inline const char * name<mocap4r2_msgs::srv::CreateRigidBody_Response>()
{
  return "mocap4r2_msgs/srv/CreateRigidBody_Response";
}

template<>
struct has_fixed_size<mocap4r2_msgs::srv::CreateRigidBody_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mocap4r2_msgs::srv::CreateRigidBody_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mocap4r2_msgs::srv::CreateRigidBody_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mocap4r2_msgs::srv::CreateRigidBody>()
{
  return "mocap4r2_msgs::srv::CreateRigidBody";
}

template<>
inline const char * name<mocap4r2_msgs::srv::CreateRigidBody>()
{
  return "mocap4r2_msgs/srv/CreateRigidBody";
}

template<>
struct has_fixed_size<mocap4r2_msgs::srv::CreateRigidBody>
  : std::integral_constant<
    bool,
    has_fixed_size<mocap4r2_msgs::srv::CreateRigidBody_Request>::value &&
    has_fixed_size<mocap4r2_msgs::srv::CreateRigidBody_Response>::value
  >
{
};

template<>
struct has_bounded_size<mocap4r2_msgs::srv::CreateRigidBody>
  : std::integral_constant<
    bool,
    has_bounded_size<mocap4r2_msgs::srv::CreateRigidBody_Request>::value &&
    has_bounded_size<mocap4r2_msgs::srv::CreateRigidBody_Response>::value
  >
{
};

template<>
struct is_service<mocap4r2_msgs::srv::CreateRigidBody>
  : std::true_type
{
};

template<>
struct is_service_request<mocap4r2_msgs::srv::CreateRigidBody_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mocap4r2_msgs::srv::CreateRigidBody_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__TRAITS_HPP_
