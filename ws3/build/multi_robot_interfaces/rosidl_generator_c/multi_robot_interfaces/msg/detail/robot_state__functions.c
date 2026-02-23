// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from multi_robot_interfaces:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "multi_robot_interfaces/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot_id`
#include "rosidl_runtime_c/string_functions.h"

bool
multi_robot_interfaces__msg__RobotState__init(multi_robot_interfaces__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    multi_robot_interfaces__msg__RobotState__fini(msg);
    return false;
  }
  // x
  // y
  // theta
  return true;
}

void
multi_robot_interfaces__msg__RobotState__fini(multi_robot_interfaces__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // x
  // y
  // theta
}

bool
multi_robot_interfaces__msg__RobotState__are_equal(const multi_robot_interfaces__msg__RobotState * lhs, const multi_robot_interfaces__msg__RobotState * rhs)
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
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  return true;
}

bool
multi_robot_interfaces__msg__RobotState__copy(
  const multi_robot_interfaces__msg__RobotState * input,
  multi_robot_interfaces__msg__RobotState * output)
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
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // theta
  output->theta = input->theta;
  return true;
}

multi_robot_interfaces__msg__RobotState *
multi_robot_interfaces__msg__RobotState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_robot_interfaces__msg__RobotState * msg = (multi_robot_interfaces__msg__RobotState *)allocator.allocate(sizeof(multi_robot_interfaces__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(multi_robot_interfaces__msg__RobotState));
  bool success = multi_robot_interfaces__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
multi_robot_interfaces__msg__RobotState__destroy(multi_robot_interfaces__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    multi_robot_interfaces__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
multi_robot_interfaces__msg__RobotState__Sequence__init(multi_robot_interfaces__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_robot_interfaces__msg__RobotState * data = NULL;

  if (size) {
    data = (multi_robot_interfaces__msg__RobotState *)allocator.zero_allocate(size, sizeof(multi_robot_interfaces__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = multi_robot_interfaces__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        multi_robot_interfaces__msg__RobotState__fini(&data[i - 1]);
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
multi_robot_interfaces__msg__RobotState__Sequence__fini(multi_robot_interfaces__msg__RobotState__Sequence * array)
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
      multi_robot_interfaces__msg__RobotState__fini(&array->data[i]);
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

multi_robot_interfaces__msg__RobotState__Sequence *
multi_robot_interfaces__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  multi_robot_interfaces__msg__RobotState__Sequence * array = (multi_robot_interfaces__msg__RobotState__Sequence *)allocator.allocate(sizeof(multi_robot_interfaces__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = multi_robot_interfaces__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
multi_robot_interfaces__msg__RobotState__Sequence__destroy(multi_robot_interfaces__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    multi_robot_interfaces__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
multi_robot_interfaces__msg__RobotState__Sequence__are_equal(const multi_robot_interfaces__msg__RobotState__Sequence * lhs, const multi_robot_interfaces__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!multi_robot_interfaces__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
multi_robot_interfaces__msg__RobotState__Sequence__copy(
  const multi_robot_interfaces__msg__RobotState__Sequence * input,
  multi_robot_interfaces__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(multi_robot_interfaces__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    multi_robot_interfaces__msg__RobotState * data =
      (multi_robot_interfaces__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!multi_robot_interfaces__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          multi_robot_interfaces__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!multi_robot_interfaces__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
