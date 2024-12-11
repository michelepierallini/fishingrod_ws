// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pi3hat_moteus_int_msgs:msg/PacketPass.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__BUILDER_HPP_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pi3hat_moteus_int_msgs/msg/detail/packet_pass__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pi3hat_moteus_int_msgs
{

namespace msg
{

namespace builder
{

class Init_PacketPass_pack_loss
{
public:
  explicit Init_PacketPass_pack_loss(::pi3hat_moteus_int_msgs::msg::PacketPass & msg)
  : msg_(msg)
  {}
  ::pi3hat_moteus_int_msgs::msg::PacketPass pack_loss(::pi3hat_moteus_int_msgs::msg::PacketPass::_pack_loss_type arg)
  {
    msg_.pack_loss = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::PacketPass msg_;
};

class Init_PacketPass_name
{
public:
  explicit Init_PacketPass_name(::pi3hat_moteus_int_msgs::msg::PacketPass & msg)
  : msg_(msg)
  {}
  Init_PacketPass_pack_loss name(::pi3hat_moteus_int_msgs::msg::PacketPass::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_PacketPass_pack_loss(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::PacketPass msg_;
};

class Init_PacketPass_cycle_dur
{
public:
  explicit Init_PacketPass_cycle_dur(::pi3hat_moteus_int_msgs::msg::PacketPass & msg)
  : msg_(msg)
  {}
  Init_PacketPass_name cycle_dur(::pi3hat_moteus_int_msgs::msg::PacketPass::_cycle_dur_type arg)
  {
    msg_.cycle_dur = std::move(arg);
    return Init_PacketPass_name(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::PacketPass msg_;
};

class Init_PacketPass_valid
{
public:
  explicit Init_PacketPass_valid(::pi3hat_moteus_int_msgs::msg::PacketPass & msg)
  : msg_(msg)
  {}
  Init_PacketPass_cycle_dur valid(::pi3hat_moteus_int_msgs::msg::PacketPass::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_PacketPass_cycle_dur(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::PacketPass msg_;
};

class Init_PacketPass_header
{
public:
  Init_PacketPass_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PacketPass_valid header(::pi3hat_moteus_int_msgs::msg::PacketPass::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PacketPass_valid(msg_);
  }

private:
  ::pi3hat_moteus_int_msgs::msg::PacketPass msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::pi3hat_moteus_int_msgs::msg::PacketPass>()
{
  return pi3hat_moteus_int_msgs::msg::builder::Init_PacketPass_header();
}

}  // namespace pi3hat_moteus_int_msgs

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__BUILDER_HPP_
