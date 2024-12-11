// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pi3hat_moteus_int_msgs:msg/JointsStates.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_STATES__BUILDER_HPP_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pi3hat_moteus_int_msgs/msg/detail/joints_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pi3hat_moteus_int_msgs
{

namespace msg
{

namespace builder
{

class Init_JointsStates_sec_enc_vel
{
public:
  explicit Init_JointsStates_sec_enc_vel(::pi3hat_moteus_int_msgs::msg::JointsStates & msg)
  : msg_(msg)
  {}
  ::pi3hat_moteus_int_msgs::msg::JointsStates sec_enc_vel(::pi3hat_moteus_int_msgs::msg::JointsStates::_sec_enc_vel_type arg)
  {
    msg_.sec_enc_vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsStates msg_;
};

class Init_JointsStates_sec_enc_pos
{
public:
  explicit Init_JointsStates_sec_enc_pos(::pi3hat_moteus_int_msgs::msg::JointsStates & msg)
  : msg_(msg)
  {}
  Init_JointsStates_sec_enc_vel sec_enc_pos(::pi3hat_moteus_int_msgs::msg::JointsStates::_sec_enc_pos_type arg)
  {
    msg_.sec_enc_pos = std::move(arg);
    return Init_JointsStates_sec_enc_vel(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsStates msg_;
};

class Init_JointsStates_temperature
{
public:
  explicit Init_JointsStates_temperature(::pi3hat_moteus_int_msgs::msg::JointsStates & msg)
  : msg_(msg)
  {}
  Init_JointsStates_sec_enc_pos temperature(::pi3hat_moteus_int_msgs::msg::JointsStates::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_JointsStates_sec_enc_pos(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsStates msg_;
};

class Init_JointsStates_current
{
public:
  explicit Init_JointsStates_current(::pi3hat_moteus_int_msgs::msg::JointsStates & msg)
  : msg_(msg)
  {}
  Init_JointsStates_temperature current(::pi3hat_moteus_int_msgs::msg::JointsStates::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_JointsStates_temperature(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsStates msg_;
};

class Init_JointsStates_effort
{
public:
  explicit Init_JointsStates_effort(::pi3hat_moteus_int_msgs::msg::JointsStates & msg)
  : msg_(msg)
  {}
  Init_JointsStates_current effort(::pi3hat_moteus_int_msgs::msg::JointsStates::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return Init_JointsStates_current(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsStates msg_;
};

class Init_JointsStates_velocity
{
public:
  explicit Init_JointsStates_velocity(::pi3hat_moteus_int_msgs::msg::JointsStates & msg)
  : msg_(msg)
  {}
  Init_JointsStates_effort velocity(::pi3hat_moteus_int_msgs::msg::JointsStates::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_JointsStates_effort(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsStates msg_;
};

class Init_JointsStates_position
{
public:
  explicit Init_JointsStates_position(::pi3hat_moteus_int_msgs::msg::JointsStates & msg)
  : msg_(msg)
  {}
  Init_JointsStates_velocity position(::pi3hat_moteus_int_msgs::msg::JointsStates::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_JointsStates_velocity(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsStates msg_;
};

class Init_JointsStates_name
{
public:
  explicit Init_JointsStates_name(::pi3hat_moteus_int_msgs::msg::JointsStates & msg)
  : msg_(msg)
  {}
  Init_JointsStates_position name(::pi3hat_moteus_int_msgs::msg::JointsStates::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_JointsStates_position(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsStates msg_;
};

class Init_JointsStates_header
{
public:
  Init_JointsStates_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointsStates_name header(::pi3hat_moteus_int_msgs::msg::JointsStates::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointsStates_name(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::JointsStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pi3hat_moteus_int_msgs::msg::JointsStates>()
{
  return pi3hat_moteus_int_msgs::msg::builder::Init_JointsStates_header();
}

}  // namespace pi3hat_moteus_int_msgs

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__JOINTS_STATES__BUILDER_HPP_
