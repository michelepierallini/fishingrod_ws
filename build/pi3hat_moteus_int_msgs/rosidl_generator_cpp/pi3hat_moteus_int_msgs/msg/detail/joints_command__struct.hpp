// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pi3hat_moteus_int_msgs:msg/JointsCommand.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__STRUCT_HPP_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pi3hat_moteus_int_msgs__msg__JointsCommand __attribute__((deprecated))
#else
# define DEPRECATED__pi3hat_moteus_int_msgs__msg__JointsCommand __declspec(deprecated)
#endif

namespace pi3hat_moteus_int_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointsCommand_
{
  using Type = JointsCommand_<ContainerAllocator>;

  explicit JointsCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit JointsCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _name_type name;
  using _position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _position_type position;
  using _velocity_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _velocity_type velocity;
  using _effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _effort_type effort;
  using _kp_scale_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _kp_scale_type kp_scale;
  using _kd_scale_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _kd_scale_type kd_scale;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->effort = _arg;
    return *this;
  }
  Type & set__kp_scale(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->kp_scale = _arg;
    return *this;
  }
  Type & set__kd_scale(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->kd_scale = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pi3hat_moteus_int_msgs__msg__JointsCommand
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pi3hat_moteus_int_msgs__msg__JointsCommand
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::JointsCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointsCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    if (this->kp_scale != other.kp_scale) {
      return false;
    }
    if (this->kd_scale != other.kd_scale) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointsCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointsCommand_

// alias to use template instance with default allocator
using JointsCommand =
  pi3hat_moteus_int_msgs::msg::JointsCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pi3hat_moteus_int_msgs

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__STRUCT_HPP_
