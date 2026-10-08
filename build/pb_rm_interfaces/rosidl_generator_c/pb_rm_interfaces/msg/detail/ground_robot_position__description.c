// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pb_rm_interfaces:msg/GroundRobotPosition.idl
// generated code does not contain a copyright notice

#include "pb_rm_interfaces/msg/detail/ground_robot_position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pb_rm_interfaces
const rosidl_type_hash_t *
pb_rm_interfaces__msg__GroundRobotPosition__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xae, 0x80, 0x03, 0xbe, 0x3f, 0x88, 0xa0, 0xf2,
      0x20, 0x8a, 0x5f, 0x53, 0x28, 0xe1, 0x5e, 0x7e,
      0x39, 0x67, 0xc4, 0xdd, 0x6d, 0x7b, 0xeb, 0x93,
      0xc9, 0xe0, 0x92, 0x1f, 0x5a, 0x7e, 0xe3, 0x78,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/point__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
#endif

static char pb_rm_interfaces__msg__GroundRobotPosition__TYPE_NAME[] = "pb_rm_interfaces/msg/GroundRobotPosition";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";

// Define type names, field names, and default values
static char pb_rm_interfaces__msg__GroundRobotPosition__FIELD_NAME__hero_position[] = "hero_position";
static char pb_rm_interfaces__msg__GroundRobotPosition__FIELD_NAME__engineer_position[] = "engineer_position";
static char pb_rm_interfaces__msg__GroundRobotPosition__FIELD_NAME__standard_3_position[] = "standard_3_position";
static char pb_rm_interfaces__msg__GroundRobotPosition__FIELD_NAME__standard_4_position[] = "standard_4_position";

static rosidl_runtime_c__type_description__Field pb_rm_interfaces__msg__GroundRobotPosition__FIELDS[] = {
  {
    {pb_rm_interfaces__msg__GroundRobotPosition__FIELD_NAME__hero_position, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__GroundRobotPosition__FIELD_NAME__engineer_position, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__GroundRobotPosition__FIELD_NAME__standard_3_position, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__GroundRobotPosition__FIELD_NAME__standard_4_position, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription pb_rm_interfaces__msg__GroundRobotPosition__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pb_rm_interfaces__msg__GroundRobotPosition__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pb_rm_interfaces__msg__GroundRobotPosition__TYPE_NAME, 40, 40},
      {pb_rm_interfaces__msg__GroundRobotPosition__FIELDS, 4, 4},
    },
    {pb_rm_interfaces__msg__GroundRobotPosition__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe5\\x9c\\xb0\\xe9\\x9d\\xa2\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe4\\xbd\\x8d\\xe7\\xbd\\xae\\xe6\\x95\\xb0\\xe6\\x8d\\xae (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x020B)\n"
  "\n"
  "geometry_msgs/Point hero_position            # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe8\\x8b\\xb1\\xe9\\x9b\\x84\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe4\\xbd\\x8d\\xe7\\xbd\\xae\n"
  "geometry_msgs/Point engineer_position        # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\xb7\\xa5\\xe7\\xa8\\x8b\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe4\\xbd\\x8d\\xe7\\xbd\\xae\n"
  "geometry_msgs/Point standard_3_position      # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9 3 \\xe5\\x8f\\xb7\\xe6\\xad\\xa5\\xe5\\x85\\xb5\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe4\\xbd\\x8d\\xe7\\xbd\\xae\n"
  "geometry_msgs/Point standard_4_position      # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9 4 \\xe5\\x8f\\xb7\\xe6\\xad\\xa5\\xe5\\x85\\xb5\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe4\\xbd\\x8d\\xe7\\xbd\\xae";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pb_rm_interfaces__msg__GroundRobotPosition__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pb_rm_interfaces__msg__GroundRobotPosition__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 274, 274},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pb_rm_interfaces__msg__GroundRobotPosition__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pb_rm_interfaces__msg__GroundRobotPosition__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
