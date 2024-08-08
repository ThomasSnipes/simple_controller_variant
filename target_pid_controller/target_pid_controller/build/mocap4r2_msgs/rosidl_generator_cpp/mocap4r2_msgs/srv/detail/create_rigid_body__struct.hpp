// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mocap4r2_msgs:srv/CreateRigidBody.idl
// generated code does not contain a copyright notice

#ifndef MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__STRUCT_HPP_
#define MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mocap4r2_msgs__srv__CreateRigidBody_Request __attribute__((deprecated))
#else
# define DEPRECATED__mocap4r2_msgs__srv__CreateRigidBody_Request __declspec(deprecated)
#endif

namespace mocap4r2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CreateRigidBody_Request_
{
  using Type = CreateRigidBody_Request_<ContainerAllocator>;

  explicit CreateRigidBody_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rigid_body_name = "";
      this->link_parent = "";
    }
  }

  explicit CreateRigidBody_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rigid_body_name(_alloc),
    link_parent(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rigid_body_name = "";
      this->link_parent = "";
    }
  }

  // field types and members
  using _rigid_body_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _rigid_body_name_type rigid_body_name;
  using _link_parent_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _link_parent_type link_parent;
  using _markers_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _markers_type markers;

  // setters for named parameter idiom
  Type & set__rigid_body_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->rigid_body_name = _arg;
    return *this;
  }
  Type & set__link_parent(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->link_parent = _arg;
    return *this;
  }
  Type & set__markers(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->markers = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mocap4r2_msgs__srv__CreateRigidBody_Request
    std::shared_ptr<mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mocap4r2_msgs__srv__CreateRigidBody_Request
    std::shared_ptr<mocap4r2_msgs::srv::CreateRigidBody_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CreateRigidBody_Request_ & other) const
  {
    if (this->rigid_body_name != other.rigid_body_name) {
      return false;
    }
    if (this->link_parent != other.link_parent) {
      return false;
    }
    if (this->markers != other.markers) {
      return false;
    }
    return true;
  }
  bool operator!=(const CreateRigidBody_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CreateRigidBody_Request_

// alias to use template instance with default allocator
using CreateRigidBody_Request =
  mocap4r2_msgs::srv::CreateRigidBody_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mocap4r2_msgs


#ifndef _WIN32
# define DEPRECATED__mocap4r2_msgs__srv__CreateRigidBody_Response __attribute__((deprecated))
#else
# define DEPRECATED__mocap4r2_msgs__srv__CreateRigidBody_Response __declspec(deprecated)
#endif

namespace mocap4r2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CreateRigidBody_Response_
{
  using Type = CreateRigidBody_Response_<ContainerAllocator>;

  explicit CreateRigidBody_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit CreateRigidBody_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mocap4r2_msgs__srv__CreateRigidBody_Response
    std::shared_ptr<mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mocap4r2_msgs__srv__CreateRigidBody_Response
    std::shared_ptr<mocap4r2_msgs::srv::CreateRigidBody_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CreateRigidBody_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const CreateRigidBody_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CreateRigidBody_Response_

// alias to use template instance with default allocator
using CreateRigidBody_Response =
  mocap4r2_msgs::srv::CreateRigidBody_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mocap4r2_msgs

namespace mocap4r2_msgs
{

namespace srv
{

struct CreateRigidBody
{
  using Request = mocap4r2_msgs::srv::CreateRigidBody_Request;
  using Response = mocap4r2_msgs::srv::CreateRigidBody_Response;
};

}  // namespace srv

}  // namespace mocap4r2_msgs

#endif  // MOCAP4R2_MSGS__SRV__DETAIL__CREATE_RIGID_BODY__STRUCT_HPP_
