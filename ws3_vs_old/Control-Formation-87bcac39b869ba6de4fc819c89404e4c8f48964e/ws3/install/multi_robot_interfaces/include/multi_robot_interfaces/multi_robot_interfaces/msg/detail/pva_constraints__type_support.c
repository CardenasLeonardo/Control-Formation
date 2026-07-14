// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from multi_robot_interfaces:msg/PVAConstraints.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "multi_robot_interfaces/msg/detail/pva_constraints__rosidl_typesupport_introspection_c.h"
#include "multi_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "multi_robot_interfaces/msg/detail/pva_constraints__functions.h"
#include "multi_robot_interfaces/msg/detail/pva_constraints__struct.h"


// Include directives for member types
// Member `robot_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `a`
// Member `b`
// Member `c`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  multi_robot_interfaces__msg__PVAConstraints__init(message_memory);
}

void multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_fini_function(void * message_memory)
{
  multi_robot_interfaces__msg__PVAConstraints__fini(message_memory);
}

size_t multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__size_function__PVAConstraints__a(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_const_function__PVAConstraints__a(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_function__PVAConstraints__a(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__fetch_function__PVAConstraints__a(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_const_function__PVAConstraints__a(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__assign_function__PVAConstraints__a(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_function__PVAConstraints__a(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__resize_function__PVAConstraints__a(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__size_function__PVAConstraints__b(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_const_function__PVAConstraints__b(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_function__PVAConstraints__b(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__fetch_function__PVAConstraints__b(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_const_function__PVAConstraints__b(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__assign_function__PVAConstraints__b(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_function__PVAConstraints__b(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__resize_function__PVAConstraints__b(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__size_function__PVAConstraints__c(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_const_function__PVAConstraints__c(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_function__PVAConstraints__c(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__fetch_function__PVAConstraints__c(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_const_function__PVAConstraints__c(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__assign_function__PVAConstraints__c(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_function__PVAConstraints__c(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__resize_function__PVAConstraints__c(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_message_member_array[8] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_robot_interfaces__msg__PVAConstraints, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_robot_interfaces__msg__PVAConstraints, a),  // bytes offset in struct
    NULL,  // default value
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__size_function__PVAConstraints__a,  // size() function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_const_function__PVAConstraints__a,  // get_const(index) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_function__PVAConstraints__a,  // get(index) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__fetch_function__PVAConstraints__a,  // fetch(index, &value) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__assign_function__PVAConstraints__a,  // assign(index, value) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__resize_function__PVAConstraints__a  // resize(index) function pointer
  },
  {
    "b",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_robot_interfaces__msg__PVAConstraints, b),  // bytes offset in struct
    NULL,  // default value
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__size_function__PVAConstraints__b,  // size() function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_const_function__PVAConstraints__b,  // get_const(index) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_function__PVAConstraints__b,  // get(index) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__fetch_function__PVAConstraints__b,  // fetch(index, &value) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__assign_function__PVAConstraints__b,  // assign(index, value) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__resize_function__PVAConstraints__b  // resize(index) function pointer
  },
  {
    "c",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_robot_interfaces__msg__PVAConstraints, c),  // bytes offset in struct
    NULL,  // default value
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__size_function__PVAConstraints__c,  // size() function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_const_function__PVAConstraints__c,  // get_const(index) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__get_function__PVAConstraints__c,  // get(index) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__fetch_function__PVAConstraints__c,  // fetch(index, &value) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__assign_function__PVAConstraints__c,  // assign(index, value) function pointer
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__resize_function__PVAConstraints__c  // resize(index) function pointer
  },
  {
    "v_goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_robot_interfaces__msg__PVAConstraints, v_goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "w_goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_robot_interfaces__msg__PVAConstraints, w_goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "v_star",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_robot_interfaces__msg__PVAConstraints, v_star),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "w_star",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(multi_robot_interfaces__msg__PVAConstraints, w_star),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_message_members = {
  "multi_robot_interfaces__msg",  // message namespace
  "PVAConstraints",  // message name
  8,  // number of fields
  sizeof(multi_robot_interfaces__msg__PVAConstraints),
  multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_message_member_array,  // message members
  multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_init_function,  // function to initialize message memory (memory has to be allocated)
  multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_message_type_support_handle = {
  0,
  &multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_multi_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, multi_robot_interfaces, msg, PVAConstraints)() {
  if (!multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_message_type_support_handle.typesupport_identifier) {
    multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &multi_robot_interfaces__msg__PVAConstraints__rosidl_typesupport_introspection_c__PVAConstraints_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
