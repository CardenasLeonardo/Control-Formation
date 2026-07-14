// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from multi_robot_interfaces:msg/PVAConstraints.idl
// generated code does not contain a copyright notice

#ifndef MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__BUILDER_HPP_
#define MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "multi_robot_interfaces/msg/detail/pva_constraints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace multi_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_PVAConstraints_w_star
{
public:
  explicit Init_PVAConstraints_w_star(::multi_robot_interfaces::msg::PVAConstraints & msg)
  : msg_(msg)
  {}
  ::multi_robot_interfaces::msg::PVAConstraints w_star(::multi_robot_interfaces::msg::PVAConstraints::_w_star_type arg)
  {
    msg_.w_star = std::move(arg);
    return std::move(msg_);
  }

private:
  ::multi_robot_interfaces::msg::PVAConstraints msg_;
};

class Init_PVAConstraints_v_star
{
public:
  explicit Init_PVAConstraints_v_star(::multi_robot_interfaces::msg::PVAConstraints & msg)
  : msg_(msg)
  {}
  Init_PVAConstraints_w_star v_star(::multi_robot_interfaces::msg::PVAConstraints::_v_star_type arg)
  {
    msg_.v_star = std::move(arg);
    return Init_PVAConstraints_w_star(msg_);
  }

private:
  ::multi_robot_interfaces::msg::PVAConstraints msg_;
};

class Init_PVAConstraints_w_goal
{
public:
  explicit Init_PVAConstraints_w_goal(::multi_robot_interfaces::msg::PVAConstraints & msg)
  : msg_(msg)
  {}
  Init_PVAConstraints_v_star w_goal(::multi_robot_interfaces::msg::PVAConstraints::_w_goal_type arg)
  {
    msg_.w_goal = std::move(arg);
    return Init_PVAConstraints_v_star(msg_);
  }

private:
  ::multi_robot_interfaces::msg::PVAConstraints msg_;
};

class Init_PVAConstraints_v_goal
{
public:
  explicit Init_PVAConstraints_v_goal(::multi_robot_interfaces::msg::PVAConstraints & msg)
  : msg_(msg)
  {}
  Init_PVAConstraints_w_goal v_goal(::multi_robot_interfaces::msg::PVAConstraints::_v_goal_type arg)
  {
    msg_.v_goal = std::move(arg);
    return Init_PVAConstraints_w_goal(msg_);
  }

private:
  ::multi_robot_interfaces::msg::PVAConstraints msg_;
};

class Init_PVAConstraints_c
{
public:
  explicit Init_PVAConstraints_c(::multi_robot_interfaces::msg::PVAConstraints & msg)
  : msg_(msg)
  {}
  Init_PVAConstraints_v_goal c(::multi_robot_interfaces::msg::PVAConstraints::_c_type arg)
  {
    msg_.c = std::move(arg);
    return Init_PVAConstraints_v_goal(msg_);
  }

private:
  ::multi_robot_interfaces::msg::PVAConstraints msg_;
};

class Init_PVAConstraints_b
{
public:
  explicit Init_PVAConstraints_b(::multi_robot_interfaces::msg::PVAConstraints & msg)
  : msg_(msg)
  {}
  Init_PVAConstraints_c b(::multi_robot_interfaces::msg::PVAConstraints::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_PVAConstraints_c(msg_);
  }

private:
  ::multi_robot_interfaces::msg::PVAConstraints msg_;
};

class Init_PVAConstraints_a
{
public:
  explicit Init_PVAConstraints_a(::multi_robot_interfaces::msg::PVAConstraints & msg)
  : msg_(msg)
  {}
  Init_PVAConstraints_b a(::multi_robot_interfaces::msg::PVAConstraints::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_PVAConstraints_b(msg_);
  }

private:
  ::multi_robot_interfaces::msg::PVAConstraints msg_;
};

class Init_PVAConstraints_robot_id
{
public:
  Init_PVAConstraints_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PVAConstraints_a robot_id(::multi_robot_interfaces::msg::PVAConstraints::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_PVAConstraints_a(msg_);
  }

private:
  ::multi_robot_interfaces::msg::PVAConstraints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::multi_robot_interfaces::msg::PVAConstraints>()
{
  return multi_robot_interfaces::msg::builder::Init_PVAConstraints_robot_id();
}

}  // namespace multi_robot_interfaces

#endif  // MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__BUILDER_HPP_
