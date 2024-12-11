// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pi3hat_moteus_int_msgs:msg/PacketPass.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__STRUCT_HPP_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__STRUCT_HPP_

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
# define DEPRECATED__pi3hat_moteus_int_msgs__msg__PacketPass __attribute__((deprecated))
#else
# define DEPRECATED__pi3hat_moteus_int_msgs__msg__PacketPass __declspec(deprecated)
#endif

namespace pi3hat_moteus_int_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PacketPass_
{
  using Type = PacketPass_<ContainerAllocator>;

  explicit PacketPass_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = 0.0;
      this->cycle_dur = 0.0;
    }
  }

  explicit PacketPass_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->valid = 0.0;
      this->cycle_dur = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _valid_type =
    double;
  _valid_type valid;
  using _cycle_dur_type =
    double;
  _cycle_dur_type cycle_dur;
  using _name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _name_type name;
  using _pack_loss_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _pack_loss_type pack_loss;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__valid(
    const double & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__cycle_dur(
    const double & _arg)
  {
    this->cycle_dur = _arg;
    return *this;
  }
  Type & set__name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__pack_loss(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->pack_loss = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator> *;
  using ConstRawPtr =
    const pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pi3hat_moteus_int_msgs__msg__PacketPass
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pi3hat_moteus_int_msgs__msg__PacketPass
    std::shared_ptr<pi3hat_moteus_int_msgs::msg::PacketPass_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PacketPass_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->cycle_dur != other.cycle_dur) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->pack_loss != other.pack_loss) {
      return false;
    }
    return true;
  }
  bool operator!=(const PacketPass_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PacketPass_

// alias to use template instance with default allocator
using PacketPass =
  pi3hat_moteus_int_msgs::msg::PacketPass_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace pi3hat_moteus_int_msgs

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__STRUCT_HPP_
