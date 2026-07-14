// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from multi_robot_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define MULTI_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "multi_robot_interfaces/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace multi_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotState_theta
{
public:
  explicit Init_RobotState_theta(::multi_robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::multi_robot_interfaces::msg::RobotState theta(::multi_robot_interfaces::msg::RobotState::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::multi_robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_y
{
public:
  explicit Init_RobotState_y(::multi_robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_theta y(::multi_robot_interfaces::msg::RobotState::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_RobotState_theta(msg_);
  }

private:
  ::multi_robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_x
{
public:
  explicit Init_RobotState_x(::multi_robot_interfaces::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_y x(::multi_robot_interfaces::msg::RobotState::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RobotState_y(msg_);
  }

private:
  ::multi_robot_interfaces::msg::RobotState msg_;
};

class Init_RobotState_robot_id
{
public:
  Init_RobotState_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_x robot_id(::multi_robot_interfaces::msg::RobotState::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotState_x(msg_);
  }

private:
  ::multi_robot_interfaces::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::multi_robot_interfaces::msg::RobotState>()
{
  return multi_robot_interfaces::msg::builder::Init_RobotState_robot_id();
}

}  // namespace multi_robot_interfaces

#endif  // MULTI_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
