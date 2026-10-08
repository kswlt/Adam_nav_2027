// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pb_rm_interfaces:msg/Gimbal.idl
// generated code does not contain a copyright notice

#include "pb_rm_interfaces/msg/detail/gimbal__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pb_rm_interfaces
const rosidl_type_hash_t *
pb_rm_interfaces__msg__Gimbal__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x46, 0xf1, 0xdc, 0x11, 0xb7, 0xf3, 0x43, 0x1d,
      0x88, 0xe5, 0xfa, 0xe7, 0xa7, 0x39, 0x41, 0x62,
      0xc0, 0xa8, 0x6b, 0x99, 0xff, 0x28, 0x85, 0xaf,
      0xd7, 0x0c, 0xe8, 0x11, 0x29, 0xc9, 0x88, 0x24,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char pb_rm_interfaces__msg__Gimbal__TYPE_NAME[] = "pb_rm_interfaces/msg/Gimbal";

// Define type names, field names, and default values
static char pb_rm_interfaces__msg__Gimbal__FIELD_NAME__pitch[] = "pitch";
static char pb_rm_interfaces__msg__Gimbal__FIELD_NAME__yaw[] = "yaw";
static char pb_rm_interfaces__msg__Gimbal__FIELD_NAME__pitch_min_range[] = "pitch_min_range";
static char pb_rm_interfaces__msg__Gimbal__FIELD_NAME__pitch_max_range[] = "pitch_max_range";
static char pb_rm_interfaces__msg__Gimbal__FIELD_NAME__yaw_min_range[] = "yaw_min_range";
static char pb_rm_interfaces__msg__Gimbal__FIELD_NAME__yaw_max_range[] = "yaw_max_range";

static rosidl_runtime_c__type_description__Field pb_rm_interfaces__msg__Gimbal__FIELDS[] = {
  {
    {pb_rm_interfaces__msg__Gimbal__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Gimbal__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Gimbal__FIELD_NAME__pitch_min_range, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Gimbal__FIELD_NAME__pitch_max_range, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Gimbal__FIELD_NAME__yaw_min_range, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Gimbal__FIELD_NAME__yaw_max_range, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pb_rm_interfaces__msg__Gimbal__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pb_rm_interfaces__msg__Gimbal__TYPE_NAME, 27, 27},
      {pb_rm_interfaces__msg__Gimbal__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# msg for Gimbal, pitch and yaw.\n"
  "float32 pitch\n"
  "float32 yaw\n"
  "\n"
  "# Only used in velocity control type\n"
  "float32 pitch_min_range\n"
  "float32 pitch_max_range\n"
  "float32 yaw_min_range\n"
  "float32 yaw_max_range";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pb_rm_interfaces__msg__Gimbal__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pb_rm_interfaces__msg__Gimbal__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 188, 188},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pb_rm_interfaces__msg__Gimbal__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pb_rm_interfaces__msg__Gimbal__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
