// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pb_rm_interfaces:msg/GimbalCmd.idl
// generated code does not contain a copyright notice

#include "pb_rm_interfaces/msg/detail/gimbal_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pb_rm_interfaces
const rosidl_type_hash_t *
pb_rm_interfaces__msg__GimbalCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf8, 0xe6, 0x80, 0x73, 0x0e, 0x5c, 0xfc, 0x0e,
      0x04, 0xd5, 0x43, 0x9d, 0x27, 0x30, 0xc0, 0x31,
      0xb6, 0x2d, 0x7b, 0x87, 0x8e, 0x7f, 0x72, 0x1e,
      0xeb, 0xeb, 0x58, 0xcb, 0x2a, 0xf2, 0x2d, 0xba,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "pb_rm_interfaces/msg/detail/gimbal__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t pb_rm_interfaces__msg__Gimbal__EXPECTED_HASH = {1, {
    0x46, 0xf1, 0xdc, 0x11, 0xb7, 0xf3, 0x43, 0x1d,
    0x88, 0xe5, 0xfa, 0xe7, 0xa7, 0x39, 0x41, 0x62,
    0xc0, 0xa8, 0x6b, 0x99, 0xff, 0x28, 0x85, 0xaf,
    0xd7, 0x0c, 0xe8, 0x11, 0x29, 0xc9, 0x88, 0x24,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char pb_rm_interfaces__msg__GimbalCmd__TYPE_NAME[] = "pb_rm_interfaces/msg/GimbalCmd";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char pb_rm_interfaces__msg__Gimbal__TYPE_NAME[] = "pb_rm_interfaces/msg/Gimbal";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__header[] = "header";
static char pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__yaw_type[] = "yaw_type";
static char pb_rm_interfaces__msg__GimbalCmd__DEFAULT_VALUE__yaw_type[] = "1";
static char pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__pitch_type[] = "pitch_type";
static char pb_rm_interfaces__msg__GimbalCmd__DEFAULT_VALUE__pitch_type[] = "1";
static char pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__position[] = "position";
static char pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__velocity[] = "velocity";

static rosidl_runtime_c__type_description__Field pb_rm_interfaces__msg__GimbalCmd__FIELDS[] = {
  {
    {pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__yaw_type, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {pb_rm_interfaces__msg__GimbalCmd__DEFAULT_VALUE__yaw_type, 1, 1},
  },
  {
    {pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__pitch_type, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {pb_rm_interfaces__msg__GimbalCmd__DEFAULT_VALUE__pitch_type, 1, 1},
  },
  {
    {pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {pb_rm_interfaces__msg__Gimbal__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__GimbalCmd__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {pb_rm_interfaces__msg__Gimbal__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription pb_rm_interfaces__msg__GimbalCmd__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Gimbal__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pb_rm_interfaces__msg__GimbalCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pb_rm_interfaces__msg__GimbalCmd__TYPE_NAME, 30, 30},
      {pb_rm_interfaces__msg__GimbalCmd__FIELDS, 5, 5},
    },
    {pb_rm_interfaces__msg__GimbalCmd__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&pb_rm_interfaces__msg__Gimbal__EXPECTED_HASH, pb_rm_interfaces__msg__Gimbal__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = pb_rm_interfaces__msg__Gimbal__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "# constants for control type\n"
  "uint8 ABSOLUTE_ANGLE = 1    # position control, set position by absolute angle\n"
  "uint8 VELOCITY = 2          # velocity control, set velocity\n"
  "\n"
  "# control type\n"
  "uint8 yaw_type 1\n"
  "uint8 pitch_type 1\n"
  "\n"
  "# control dada\n"
  "Gimbal position\n"
  "Gimbal velocity";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pb_rm_interfaces__msg__GimbalCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pb_rm_interfaces__msg__GimbalCmd__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 292, 292},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pb_rm_interfaces__msg__GimbalCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pb_rm_interfaces__msg__GimbalCmd__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *pb_rm_interfaces__msg__Gimbal__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
