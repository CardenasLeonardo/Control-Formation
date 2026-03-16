// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from multi_robot_interfaces:msg/PVAConstraints.idl
// generated code does not contain a copyright notice
#include "multi_robot_interfaces/msg/detail/pva_constraints__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `a`
// Member `b`
// Member `c`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
multi_robot_interfaces__msg__PVAConstraints__init(multi_robot_interfaces__msg__PVAConstraints * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    multi_robot_interfaces__msg__PVAConstraints__fini(msg);
    return false;
  }
  // a
  if (!rosidl_runtime_c__double__Sequence__init(&msg->a, 0)) {
    multi_robot_interfaces__msg__PVAConstraints__fini(msg);
    return false;
  }
  // b
  if (!rosidl_runtime_c__double__Sequence__init(&msg->b, 0)) {
    multi_robot_interfaces__msg__PVAConstraints__fini(msg);
    return false;
  }
  // c
  if (!rosidl_runtime_c__double__Sequence__init(&msg->c, 0)) {
    multi_robot_interfaces__msg__PVAConstraints__fini(msg);
    return false;
  }
  // v_goal
  // w_goal
  // v_star
  // w_star
  return true;
}

void
multi_robot_interfaces__msg__PVAConstraints__fini(multi_robot_interfaces__msg__PVAConstraints * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // a
  rosidl_runtime_c__double__Sequence__fini(&msg->a);
  // b
  rosidl_runtime_c__double__Sequence__fini(&msg->b);
  // c
  rosidl_runtime_c__double__Sequence__fini(&msg->c);
  // v_goal
  // w_goal
  // v_star
  // w_star
}

bool
multi_robot_interfaces__msg__PVAConstraints__are_equal(const multi_robot_interfaces__msg__PVAConstraints * lhs, const multi_robot_interfaces__msg__PVAConstraints * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_id), &(rhs->robot_id)))
  {
    return false;
  }
  // a
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->a), &(rhs->a)))
  {
    return false;
  }
  // b
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->b), &(rhs->b)))
  {
    return false;
  }
  // c
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->c), &(rhs->c)))
  {
    return false;
  }
  // v_goal
  if (lhs->v_goal != rhs->v_goal) {
    return false;
  }
  // w_goal
  if (lhs->w_goal != rhs->w_goal) {
    return false;
  }
  // v_star
  if (lhs->v_star != rhs->v_star) {
    return false;
  }
  // w_star
  if (lhs->w_star != rhs->w_star) {
    return false;
  }
  return true;
}

bool
multi_robot_interfaces__msg__PVAConstraints__copy(
  const multi_robot_interfaces__msg__PVAConstraints * input,
  multi_robot_interfaces__msg__PVAConstraints * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_id), &(output->robot_id)))
  {
    return false;
  }
  // a
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->a), &(output->a)))
  {
    return false;
  }
  // b
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->b), &(output->b)))
  {
    return false;
  }
  // c
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->c), &(output->c)))
  {
    return false;
  }
  // v_goal
  output->v_goal = input->v_goal;
  // w_goal
  output->w_goal = input->w_goal;
  // v_star
  output->v_star = input->v_star;
  // w_star
  output->w_star = input->w_star;
  return true;
}

multi_robot_interfaces__msg__PVAConstraints *
multi_robot_interfaces__msg__PVAConstraints__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_robot_interfaces__msg__PVAConstraints * msg = (multi_robot_interfaces__msg__PVAConstraints *)allocator.allocate(sizeof(multi_robot_interfaces__msg__PVAConstraints), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(multi_robot_interfaces__msg__PVAConstraints));
  bool success = multi_robot_interfaces__msg__PVAConstraints__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
multi_robot_interfaces__msg__PVAConstraints__destroy(multi_robot_interfaces__msg__PVAConstraints * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    multi_robot_interfaces__msg__PVAConstraints__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
multi_robot_interfaces__msg__PVAConstraints__Sequence__init(multi_robot_interfaces__msg__PVAConstraints__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_robot_interfaces__msg__PVAConstraints * data = NULL;

  if (size) {
    data = (multi_robot_interfaces__msg__PVAConstraints *)allocator.zero_allocate(size, sizeof(multi_robot_interfaces__msg__PVAConstraints), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = multi_robot_interfaces__msg__PVAConstraints__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        multi_robot_interfaces__msg__PVAConstraints__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
multi_robot_interfaces__msg__PVAConstraints__Sequence__fini(multi_robot_interfaces__msg__PVAConstraints__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      multi_robot_interfaces__msg__PVAConstraints__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

multi_robot_interfaces__msg__PVAConstraints__Sequence *
multi_robot_interfaces__msg__PVAConstraints__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_robot_interfaces__msg__PVAConstraints__Sequence * array = (multi_robot_interfaces__msg__PVAConstraints__Sequence *)allocator.allocate(sizeof(multi_robot_interfaces__msg__PVAConstraints__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = multi_robot_interfaces__msg__PVAConstraints__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
multi_robot_interfaces__msg__PVAConstraints__Sequence__destroy(multi_robot_interfaces__msg__PVAConstraints__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    multi_robot_interfaces__msg__PVAConstraints__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
multi_robot_interfaces__msg__PVAConstraints__Sequence__are_equal(const multi_robot_interfaces__msg__PVAConstraints__Sequence * lhs, const multi_robot_interfaces__msg__PVAConstraints__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!multi_robot_interfaces__msg__PVAConstraints__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
multi_robot_interfaces__msg__PVAConstraints__Sequence__copy(
  const multi_robot_interfaces__msg__PVAConstraints__Sequence * input,
  multi_robot_interfaces__msg__PVAConstraints__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(multi_robot_interfaces__msg__PVAConstraints);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    multi_robot_interfaces__msg__PVAConstraints * data =
      (multi_robot_interfaces__msg__PVAConstraints *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!multi_robot_interfaces__msg__PVAConstraints__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          multi_robot_interfaces__msg__PVAConstraints__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!multi_robot_interfaces__msg__PVAConstraints__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
