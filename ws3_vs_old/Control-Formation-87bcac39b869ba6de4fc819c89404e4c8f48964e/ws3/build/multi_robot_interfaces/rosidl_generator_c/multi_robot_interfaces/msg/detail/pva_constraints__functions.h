// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from multi_robot_interfaces:msg/PVAConstraints.idl
// generated code does not contain a copyright notice

#ifndef MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__FUNCTIONS_H_
#define MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "multi_robot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "multi_robot_interfaces/msg/detail/pva_constraints__struct.h"

/// Initialize msg/PVAConstraints message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * multi_robot_interfaces__msg__PVAConstraints
 * )) before or use
 * multi_robot_interfaces__msg__PVAConstraints__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
bool
multi_robot_interfaces__msg__PVAConstraints__init(multi_robot_interfaces__msg__PVAConstraints * msg);

/// Finalize msg/PVAConstraints message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
void
multi_robot_interfaces__msg__PVAConstraints__fini(multi_robot_interfaces__msg__PVAConstraints * msg);

/// Create msg/PVAConstraints message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * multi_robot_interfaces__msg__PVAConstraints__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
multi_robot_interfaces__msg__PVAConstraints *
multi_robot_interfaces__msg__PVAConstraints__create();

/// Destroy msg/PVAConstraints message.
/**
 * It calls
 * multi_robot_interfaces__msg__PVAConstraints__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
void
multi_robot_interfaces__msg__PVAConstraints__destroy(multi_robot_interfaces__msg__PVAConstraints * msg);

/// Check for msg/PVAConstraints message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
bool
multi_robot_interfaces__msg__PVAConstraints__are_equal(const multi_robot_interfaces__msg__PVAConstraints * lhs, const multi_robot_interfaces__msg__PVAConstraints * rhs);

/// Copy a msg/PVAConstraints message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
bool
multi_robot_interfaces__msg__PVAConstraints__copy(
  const multi_robot_interfaces__msg__PVAConstraints * input,
  multi_robot_interfaces__msg__PVAConstraints * output);

/// Initialize array of msg/PVAConstraints messages.
/**
 * It allocates the memory for the number of elements and calls
 * multi_robot_interfaces__msg__PVAConstraints__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
bool
multi_robot_interfaces__msg__PVAConstraints__Sequence__init(multi_robot_interfaces__msg__PVAConstraints__Sequence * array, size_t size);

/// Finalize array of msg/PVAConstraints messages.
/**
 * It calls
 * multi_robot_interfaces__msg__PVAConstraints__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
void
multi_robot_interfaces__msg__PVAConstraints__Sequence__fini(multi_robot_interfaces__msg__PVAConstraints__Sequence * array);

/// Create array of msg/PVAConstraints messages.
/**
 * It allocates the memory for the array and calls
 * multi_robot_interfaces__msg__PVAConstraints__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
multi_robot_interfaces__msg__PVAConstraints__Sequence *
multi_robot_interfaces__msg__PVAConstraints__Sequence__create(size_t size);

/// Destroy array of msg/PVAConstraints messages.
/**
 * It calls
 * multi_robot_interfaces__msg__PVAConstraints__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
void
multi_robot_interfaces__msg__PVAConstraints__Sequence__destroy(multi_robot_interfaces__msg__PVAConstraints__Sequence * array);

/// Check for msg/PVAConstraints message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
bool
multi_robot_interfaces__msg__PVAConstraints__Sequence__are_equal(const multi_robot_interfaces__msg__PVAConstraints__Sequence * lhs, const multi_robot_interfaces__msg__PVAConstraints__Sequence * rhs);

/// Copy an array of msg/PVAConstraints messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_multi_robot_interfaces
bool
multi_robot_interfaces__msg__PVAConstraints__Sequence__copy(
  const multi_robot_interfaces__msg__PVAConstraints__Sequence * input,
  multi_robot_interfaces__msg__PVAConstraints__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MULTI_ROBOT_INTERFACES__MSG__DETAIL__PVA_CONSTRAINTS__FUNCTIONS_H_
