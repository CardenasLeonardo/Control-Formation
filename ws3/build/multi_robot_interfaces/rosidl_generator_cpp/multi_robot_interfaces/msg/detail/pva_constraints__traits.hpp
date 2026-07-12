// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from multi_robot_interfaces:msg/PVAConstraints.idl
// generated code does not contain a copyright notice

#ifndef MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__TRAITS_HPP_
#define MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "multi_robot_interfaces/msg/detail/pva_constraints__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace multi_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PVAConstraints & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: a
  {
    if (msg.a.size() == 0) {
      out << "a: []";
    } else {
      out << "a: [";
      size_t pending_items = msg.a.size();
      for (auto item : msg.a) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: b
  {
    if (msg.b.size() == 0) {
      out << "b: []";
    } else {
      out << "b: [";
      size_t pending_items = msg.b.size();
      for (auto item : msg.b) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: c
  {
    if (msg.c.size() == 0) {
      out << "c: []";
    } else {
      out << "c: [";
      size_t pending_items = msg.c.size();
      for (auto item : msg.c) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: v_goal
  {
    out << "v_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.v_goal, out);
    out << ", ";
  }

  // member: w_goal
  {
    out << "w_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.w_goal, out);
    out << ", ";
  }

  // member: v_star
  {
    out << "v_star: ";
    rosidl_generator_traits::value_to_yaml(msg.v_star, out);
    out << ", ";
  }

  // member: w_star
  {
    out << "w_star: ";
    rosidl_generator_traits::value_to_yaml(msg.w_star, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: search_dir
  {
    out << "search_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.search_dir, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PVAConstraints & msg,
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

  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.a.size() == 0) {
      out << "a: []\n";
    } else {
      out << "a:\n";
      for (auto item : msg.a) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.b.size() == 0) {
      out << "b: []\n";
    } else {
      out << "b:\n";
      for (auto item : msg.b) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.c.size() == 0) {
      out << "c: []\n";
    } else {
      out << "c:\n";
      for (auto item : msg.c) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: v_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.v_goal, out);
    out << "\n";
  }

  // member: w_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "w_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.w_goal, out);
    out << "\n";
  }

  // member: v_star
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_star: ";
    rosidl_generator_traits::value_to_yaml(msg.v_star, out);
    out << "\n";
  }

  // member: w_star
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "w_star: ";
    rosidl_generator_traits::value_to_yaml(msg.w_star, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: search_dir
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "search_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.search_dir, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PVAConstraints & msg, bool use_flow_style = false)
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
  const multi_robot_interfaces::msg::PVAConstraints & msg,
  std::ostream & out, size_t indentation = 0)
{
  multi_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use multi_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const multi_robot_interfaces::msg::PVAConstraints & msg)
{
  return multi_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<multi_robot_interfaces::msg::PVAConstraints>()
{
  return "multi_robot_interfaces::msg::PVAConstraints";
}

template<>
inline const char * name<multi_robot_interfaces::msg::PVAConstraints>()
{
  return "multi_robot_interfaces/msg/PVAConstraints";
}

template<>
struct has_fixed_size<multi_robot_interfaces::msg::PVAConstraints>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<multi_robot_interfaces::msg::PVAConstraints>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<multi_robot_interfaces::msg::PVAConstraints>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__TRAITS_HPP_
