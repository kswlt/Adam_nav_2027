// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from pb_rm_interfaces:msg/RobotStateInfo.idl
// generated code does not contain a copyright notice
#include "pb_rm_interfaces/msg/detail/robot_state_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "pb_rm_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pb_rm_interfaces/msg/detail/robot_state_info__struct.h"
#include "pb_rm_interfaces/msg/detail/robot_state_info__functions.h"
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

#include "pb_rm_interfaces/msg/detail/models__functions.h"  // models
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

bool cdr_serialize_pb_rm_interfaces__msg__Models(
  const pb_rm_interfaces__msg__Models * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_pb_rm_interfaces__msg__Models(
  eprosima::fastcdr::Cdr & cdr,
  pb_rm_interfaces__msg__Models * ros_message);

size_t get_serialized_size_pb_rm_interfaces__msg__Models(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_pb_rm_interfaces__msg__Models(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_pb_rm_interfaces__msg__Models(
  const pb_rm_interfaces__msg__Models * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_pb_rm_interfaces__msg__Models(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_pb_rm_interfaces__msg__Models(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pb_rm_interfaces, msg, Models)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pb_rm_interfaces
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pb_rm_interfaces
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pb_rm_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pb_rm_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pb_rm_interfaces
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pb_rm_interfaces
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pb_rm_interfaces
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_pb_rm_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _RobotStateInfo__ros_msg_type = pb_rm_interfaces__msg__RobotStateInfo;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pb_rm_interfaces
bool cdr_serialize_pb_rm_interfaces__msg__RobotStateInfo(
  const pb_rm_interfaces__msg__RobotStateInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: models
  {
    cdr_serialize_pb_rm_interfaces__msg__Models(
      &ros_message->models, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pb_rm_interfaces
bool cdr_deserialize_pb_rm_interfaces__msg__RobotStateInfo(
  eprosima::fastcdr::Cdr & cdr,
  pb_rm_interfaces__msg__RobotStateInfo * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: models
  {
    cdr_deserialize_pb_rm_interfaces__msg__Models(cdr, &ros_message->models);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pb_rm_interfaces
size_t get_serialized_size_pb_rm_interfaces__msg__RobotStateInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RobotStateInfo__ros_msg_type * ros_message = static_cast<const _RobotStateInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: models
  current_alignment += get_serialized_size_pb_rm_interfaces__msg__Models(
    &(ros_message->models), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pb_rm_interfaces
size_t max_serialized_size_pb_rm_interfaces__msg__RobotStateInfo(
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

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: models
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_pb_rm_interfaces__msg__Models(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pb_rm_interfaces__msg__RobotStateInfo;
    is_plain =
      (
      offsetof(DataType, models) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pb_rm_interfaces
bool cdr_serialize_key_pb_rm_interfaces__msg__RobotStateInfo(
  const pb_rm_interfaces__msg__RobotStateInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: models
  {
    cdr_serialize_key_pb_rm_interfaces__msg__Models(
      &ros_message->models, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pb_rm_interfaces
size_t get_serialized_size_key_pb_rm_interfaces__msg__RobotStateInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RobotStateInfo__ros_msg_type * ros_message = static_cast<const _RobotStateInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: models
  current_alignment += get_serialized_size_key_pb_rm_interfaces__msg__Models(
    &(ros_message->models), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pb_rm_interfaces
size_t max_serialized_size_key_pb_rm_interfaces__msg__RobotStateInfo(
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
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: models
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_pb_rm_interfaces__msg__Models(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = pb_rm_interfaces__msg__RobotStateInfo;
    is_plain =
      (
      offsetof(DataType, models) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _RobotStateInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const pb_rm_interfaces__msg__RobotStateInfo * ros_message = static_cast<const pb_rm_interfaces__msg__RobotStateInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_pb_rm_interfaces__msg__RobotStateInfo(ros_message, cdr);
}

static bool _RobotStateInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  pb_rm_interfaces__msg__RobotStateInfo * ros_message = static_cast<pb_rm_interfaces__msg__RobotStateInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_pb_rm_interfaces__msg__RobotStateInfo(cdr, ros_message);
}

static uint32_t _RobotStateInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pb_rm_interfaces__msg__RobotStateInfo(
      untyped_ros_message, 0));
}

static size_t _RobotStateInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pb_rm_interfaces__msg__RobotStateInfo(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RobotStateInfo = {
  "pb_rm_interfaces::msg",
  "RobotStateInfo",
  _RobotStateInfo__cdr_serialize,
  _RobotStateInfo__cdr_deserialize,
  _RobotStateInfo__get_serialized_size,
  _RobotStateInfo__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _RobotStateInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RobotStateInfo,
  get_message_typesupport_handle_function,
  &pb_rm_interfaces__msg__RobotStateInfo__get_type_hash,
  &pb_rm_interfaces__msg__RobotStateInfo__get_type_description,
  &pb_rm_interfaces__msg__RobotStateInfo__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pb_rm_interfaces, msg, RobotStateInfo)() {
  return &_RobotStateInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
