// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from multi_robot_interfaces:msg/PVAConstraints.idl
// generated code does not contain a copyright notice

#ifndef MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__STRUCT_H_
#define MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__STRUCT_H_

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
// Member 'a'
// Member 'b'
// Member 'c'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/PVAConstraints in the package multi_robot_interfaces.
typedef struct multi_robot_interfaces__msg__PVAConstraints
{
  rosidl_runtime_c__String robot_id;
  rosidl_runtime_c__double__Sequence a;
  rosidl_runtime_c__double__Sequence b;
  rosidl_runtime_c__double__Sequence c;
  double v_goal;
  double w_goal;
  double v_star;
  double w_star;
} multi_robot_interfaces__msg__PVAConstraints;

// Struct for a sequence of multi_robot_interfaces__msg__PVAConstraints.
typedef struct multi_robot_interfaces__msg__PVAConstraints__Sequence
{
  multi_robot_interfaces__msg__PVAConstraints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} multi_robot_interfaces__msg__PVAConstraints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__STRUCT_H_
