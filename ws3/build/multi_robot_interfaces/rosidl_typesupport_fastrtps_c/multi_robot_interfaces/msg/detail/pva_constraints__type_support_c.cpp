// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from multi_robot_interfaces:msg/PVAConstraints.idl
// generated code does not contain a copyright notice
#include "multi_robot_interfaces/msg/detail/pva_constraints__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "multi_robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "multi_robot_interfaces/msg/detail/pva_constraints__struct.h"
#include "multi_robot_interfaces/msg/detail/pva_constraints__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // a, b, c
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // a, b, c
#include "rosidl_runtime_c/string.h"  // robot_id
#include "rosidl_runtime_c/string_functions.h"  // robot_id

// forward declare type support functions


using _PVAConstraints__ros_msg_type = multi_robot_interfaces__msg__PVAConstraints;

static bool _PVAConstraints__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PVAConstraints__ros_msg_type * ros_message = static_cast<const _PVAConstraints__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    const rosidl_runtime_c__String * str = &ros_message->robot_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: a
  {
    size_t size = ros_message->a.size;
    auto array_ptr = ros_message->a.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: b
  {
    size_t size = ros_message->b.size;
    auto array_ptr = ros_message->b.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: c
  {
    size_t size = ros_message->c.size;
    auto array_ptr = ros_message->c.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: v_goal
  {
    cdr << ros_message->v_goal;
  }

  // Field name: w_goal
  {
    cdr << ros_message->w_goal;
  }

  // Field name: v_star
  {
    cdr << ros_message->v_star;
  }

  // Field name: w_star
  {
    cdr << ros_message->w_star;
  }

  return true;
}

static bool _PVAConstraints__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PVAConstraints__ros_msg_type * ros_message = static_cast<_PVAConstraints__ros_msg_type *>(untyped_ros_message);
  // Field name: robot_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->robot_id.data) {
      rosidl_runtime_c__String__init(&ros_message->robot_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->robot_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'robot_id'\n");
      return false;
    }
  }

  // Field name: a
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->a.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->a);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->a, size)) {
      fprintf(stderr, "failed to create array for field 'a'");
      return false;
    }
    auto array_ptr = ros_message->a.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: b
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->b.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->b);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->b, size)) {
      fprintf(stderr, "failed to create array for field 'b'");
      return false;
    }
    auto array_ptr = ros_message->b.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: c
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->c.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->c);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->c, size)) {
      fprintf(stderr, "failed to create array for field 'c'");
      return false;
    }
    auto array_ptr = ros_message->c.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: v_goal
  {
    cdr >> ros_message->v_goal;
  }

  // Field name: w_goal
  {
    cdr >> ros_message->w_goal;
  }

  // Field name: v_star
  {
    cdr >> ros_message->v_star;
  }

  // Field name: w_star
  {
    cdr >> ros_message->w_star;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_multi_robot_interfaces
size_t get_serialized_size_multi_robot_interfaces__msg__PVAConstraints(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PVAConstraints__ros_msg_type * ros_message = static_cast<const _PVAConstraints__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name robot_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_id.size + 1);
  // field.name a
  {
    size_t array_size = ros_message->a.size;
    auto array_ptr = ros_message->a.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name b
  {
    size_t array_size = ros_message->b.size;
    auto array_ptr = ros_message->b.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name c
  {
    size_t array_size = ros_message->c.size;
    auto array_ptr = ros_message->c.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name v_goal
  {
    size_t item_size = sizeof(ros_message->v_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name w_goal
  {
    size_t item_size = sizeof(ros_message->w_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name v_star
  {
    size_t item_size = sizeof(ros_message->v_star);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name w_star
  {
    size_t item_size = sizeof(ros_message->w_star);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PVAConstraints__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_multi_robot_interfaces__msg__PVAConstraints(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_multi_robot_interfaces
size_t max_serialized_size_multi_robot_interfaces__msg__PVAConstraints(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: robot_id
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: a
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: b
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: c
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: v_goal
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: w_goal
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: v_star
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: w_star
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = multi_robot_interfaces__msg__PVAConstraints;
    is_plain =
      (
      offsetof(DataType, w_star) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PVAConstraints__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_multi_robot_interfaces__msg__PVAConstraints(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PVAConstraints = {
  "multi_robot_interfaces::msg",
  "PVAConstraints",
  _PVAConstraints__cdr_serialize,
  _PVAConstraints__cdr_deserialize,
  _PVAConstraints__get_serialized_size,
  _PVAConstraints__max_serialized_size
};

static rosidl_message_type_support_t _PVAConstraints__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PVAConstraints,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, multi_robot_interfaces, msg, PVAConstraints)() {
  return &_PVAConstraints__type_support;
}

#if defined(__cplusplus)
}
#endif
