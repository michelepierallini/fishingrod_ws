// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pi3hat_moteus_int_msgs:msg/PacketPass.idl
// generated code does not contain a copyright notice

#ifndef PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__TRAITS_HPP_
#define PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pi3hat_moteus_int_msgs/msg/detail/packet_pass__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace pi3hat_moteus_int_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PacketPass & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << ", ";
  }

  // member: cycle_dur
  {
    out << "cycle_dur: ";
    rosidl_generator_traits::value_to_yaml(msg.cycle_dur, out);
    out << ", ";
  }

  // member: name
  {
    if (msg.name.size() == 0) {
      out << "name: []";
    } else {
      out << "name: [";
      size_t pending_items = msg.name.size();
      for (auto item : msg.name) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pack_loss
  {
    if (msg.pack_loss.size() == 0) {
      out << "pack_loss: []";
    } else {
      out << "pack_loss: [";
      size_t pending_items = msg.pack_loss.size();
      for (auto item : msg.pack_loss) {
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
  const PacketPass & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: cycle_dur
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cycle_dur: ";
    rosidl_generator_traits::value_to_yaml(msg.cycle_dur, out);
    out << "\n";
  }

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.name.size() == 0) {
      out << "name: []\n";
    } else {
      out << "name:\n";
      for (auto item : msg.name) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pack_loss
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pack_loss.size() == 0) {
      out << "pack_loss: []\n";
    } else {
      out << "pack_loss:\n";
      for (auto item : msg.pack_loss) {
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

inline std::string to_yaml(const PacketPass & msg, bool use_flow_style = false)
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

}  // namespace pi3hat_moteus_int_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pi3hat_moteus_int_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pi3hat_moteus_int_msgs::msg::PacketPass & msg,
  std::ostream & out, size_t indentation = 0)
{
  pi3hat_moteus_int_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pi3hat_moteus_int_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const pi3hat_moteus_int_msgs::msg::PacketPass & msg)
{
  return pi3hat_moteus_int_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<pi3hat_moteus_int_msgs::msg::PacketPass>()
{
  return "pi3hat_moteus_int_msgs::msg::PacketPass";
}

template<>
inline const char * name<pi3hat_moteus_int_msgs::msg::PacketPass>()
{
  return "pi3hat_moteus_int_msgs/msg/PacketPass";
}

template<>
struct has_fixed_size<pi3hat_moteus_int_msgs::msg::PacketPass>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pi3hat_moteus_int_msgs::msg::PacketPass>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pi3hat_moteus_int_msgs::msg::PacketPass>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PI3HAT_MOTEUS_INT_MSGS__MSG__DETAIL__PACKET_PASS__TRAITS_HPP_
