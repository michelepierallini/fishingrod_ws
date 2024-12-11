// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pi3hat_moteus_int_msgs:msg/JointsCommand.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__BUILDER_HPP_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pi3hat_moteus_int_msgs/msg/detail/joints_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pi3hat_moteus_int_msgs
{

namespace msg
{

namespace builder
{

class Init_JointsCommand_kd_scale
{
public:
  explicit Init_JointsCommand_kd_scale(::pi3hat_moteus_int_msgs::msg::JointsCommand & msg)
  : msg_(msg)
  {}
  ::pi3hat_moteus_int_msgs::msg::JointsCommand kd_scale(::pi3hat_moteus_int_msgs::msg::JointsCommand::_kd_scale_type arg)
  {
    msg_.kd_scale = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsCommand msg_;
};

class Init_JointsCommand_kp_scale
{
public:
  explicit Init_JointsCommand_kp_scale(::pi3hat_moteus_int_msgs::msg::JointsCommand & msg)
  : msg_(msg)
  {}
  Init_JointsCommand_kd_scale kp_scale(::pi3hat_moteus_int_msgs::msg::JointsCommand::_kp_scale_type arg)
  {
    msg_.kp_scale = std::move(arg);
    return Init_JointsCommand_kd_scale(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsCommand msg_;
};

class Init_JointsCommand_effort
{
public:
  explicit Init_JointsCommand_effort(::pi3hat_moteus_int_msgs::msg::JointsCommand & msg)
  : msg_(msg)
  {}
  Init_JointsCommand_kp_scale effort(::pi3hat_moteus_int_msgs::msg::JointsCommand::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return Init_JointsCommand_kp_scale(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsCommand msg_;
};

class Init_JointsCommand_velocity
{
public:
  explicit Init_JointsCommand_velocity(::pi3hat_moteus_int_msgs::msg::JointsCommand & msg)
  : msg_(msg)
  {}
  Init_JointsCommand_effort velocity(::pi3hat_moteus_int_msgs::msg::JointsCommand::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_JointsCommand_effort(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsCommand msg_;
};

class Init_JointsCommand_position
{
public:
  explicit Init_JointsCommand_position(::pi3hat_moteus_int_msgs::msg::JointsCommand & msg)
  : msg_(msg)
  {}
  Init_JointsCommand_velocity position(::pi3hat_moteus_int_msgs::msg::JointsCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_JointsCommand_velocity(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsCommand msg_;
};

class Init_JointsCommand_name
{
public:
  explicit Init_JointsCommand_name(::pi3hat_moteus_int_msgs::msg::JointsCommand & msg)
  : msg_(msg)
  {}
  Init_JointsCommand_position name(::pi3hat_moteus_int_msgs::msg::JointsCommand::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_JointsCommand_position(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsCommand msg_;
};

class Init_JointsCommand_header
{
public:
  Init_JointsCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointsCommand_name header(::pi3hat_moteus_int_msgs::msg::JointsCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointsCommand_name(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pi3hat_moteus_int_msgs::msg::JointsCommand>()
{
  return pi3hat_moteus_int_msgs::msg::builder::Init_JointsCommand_header();
}

}  // namespace pi3hat_moteus_int_msgs

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_COMMAND__BUILDER_HPP_
