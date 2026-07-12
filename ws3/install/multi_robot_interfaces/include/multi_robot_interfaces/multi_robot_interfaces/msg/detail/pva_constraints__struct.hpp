// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from multi_robot_interfaces:msg/PVAConstraints.idl
// generated code does not contain a copyright notice

#ifndef MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__STRUCT_HPP_
#define MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__multi_robot_interfaces__msg__PVAConstraints __attribute__((deprecated))
#else
# define DEPRECATED__multi_robot_interfaces__msg__PVAConstraints __declspec(deprecated)
#endif

namespace multi_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PVAConstraints_
{
  using Type = PVAConstraints_<ContainerAllocator>;

  explicit PVAConstraints_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->v_goal = 0.0;
      this->w_goal = 0.0;
      this->v_star = 0.0;
      this->w_star = 0.0;
      this->mode = 0l;
      this->search_dir = 0l;
    }
  }

  explicit PVAConstraints_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = "";
      this->v_goal = 0.0;
      this->w_goal = 0.0;
      this->v_star = 0.0;
      this->w_star = 0.0;
      this->mode = 0l;
      this->search_dir = 0l;
    }
  }

  // field types and members
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_id_type robot_id;
  using _a_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _a_type a;
  using _b_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _b_type b;
  using _c_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _c_type c;
  using _v_goal_type =
    double;
  _v_goal_type v_goal;
  using _w_goal_type =
    double;
  _w_goal_type w_goal;
  using _v_star_type =
    double;
  _v_star_type v_star;
  using _w_star_type =
    double;
  _w_star_type w_star;
  using _mode_type =
    int32_t;
  _mode_type mode;
  using _search_dir_type =
    int32_t;
  _search_dir_type search_dir;

  // setters for named parameter idiom
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__a(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->b = _arg;
    return *this;
  }
  Type & set__c(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->c = _arg;
    return *this;
  }
  Type & set__v_goal(
    const double & _arg)
  {
    this->v_goal = _arg;
    return *this;
  }
  Type & set__w_goal(
    const double & _arg)
  {
    this->w_goal = _arg;
    return *this;
  }
  Type & set__v_star(
    const double & _arg)
  {
    this->v_star = _arg;
    return *this;
  }
  Type & set__w_star(
    const double & _arg)
  {
    this->w_star = _arg;
    return *this;
  }
  Type & set__mode(
    const int32_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__search_dir(
    const int32_t & _arg)
  {
    this->search_dir = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator> *;
  using ConstRawPtr =
    const multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__multi_robot_interfaces__msg__PVAConstraints
    std::shared_ptr<multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__multi_robot_interfaces__msg__PVAConstraints
    std::shared_ptr<multi_robot_interfaces::msg::PVAConstraints_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PVAConstraints_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    if (this->c != other.c) {
      return false;
    }
    if (this->v_goal != other.v_goal) {
      return false;
    }
    if (this->w_goal != other.w_goal) {
      return false;
    }
    if (this->v_star != other.v_star) {
      return false;
    }
    if (this->w_star != other.w_star) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->search_dir != other.search_dir) {
      return false;
    }
    return true;
  }
  bool operator!=(const PVAConstraints_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PVAConstraints_

// alias to use template instance with default allocator
using PVAConstraints =
  multi_robot_interfaces::msg::PVAConstraints_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace multi_robot_interfaces

#endif  // MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__STRUCT_HPP_
