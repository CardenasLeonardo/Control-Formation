// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from multi_robot_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef MULTI_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define MULTI_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobotState in the package multi_robot_interfaces.
typedef struct multi_robot_interfaces__msg__RobotState
{
  rosidl_runtime_c__String robot_id;
  double x;
  double y;
  double theta;
} multi_robot_interfaces__msg__RobotState;

// Struct for a sequence of multi_robot_interfaces__msg__RobotState.
typedef struct multi_robot_interfaces__msg__RobotState__Sequence
{
  multi_robot_interfaces__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} multi_robot_interfaces__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MULTI_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
