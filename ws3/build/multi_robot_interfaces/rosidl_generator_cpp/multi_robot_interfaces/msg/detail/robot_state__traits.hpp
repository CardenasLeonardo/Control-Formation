// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from multi_robot_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define MULTI_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "multi_robot_interfaces/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace multi_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
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

}  // namespace multi_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use multi_robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const multi_robot_interfaces::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  multi_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use multi_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const multi_robot_interfaces::msg::RobotState & msg)
{
  return multi_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<multi_robot_interfaces::msg::RobotState>()
{
  return "multi_robot_interfaces::msg::RobotState";
}

template<>
inline const char * name<multi_robot_interfaces::msg::RobotState>()
{
  return "multi_robot_interfaces/msg/RobotState";
}

template<>
struct has_fixed_size<multi_robot_interfaces::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<multi_robot_interfaces::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<multi_robot_interfaces::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MULTI_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
