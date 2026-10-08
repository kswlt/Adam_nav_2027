// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pb_rm_interfaces:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#include "pb_rm_interfaces/msg/detail/robot_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pb_rm_interfaces
const rosidl_type_hash_t *
pb_rm_interfaces__msg__RobotStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8a, 0x4c, 0xe1, 0x6e, 0x6e, 0x80, 0xa7, 0x11,
      0x83, 0x8f, 0x46, 0x1b, 0xe6, 0xcf, 0x0e, 0x3f,
      0x45, 0x3d, 0x87, 0x53, 0xb2, 0x09, 0xa8, 0x6c,
      0x8a, 0xe6, 0x1a, 0xfd, 0xa2, 0x27, 0xfc, 0x12,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/point__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Pose__EXPECTED_HASH = {1, {
    0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2,
    0x99, 0x69, 0x84, 0xe8, 0x12, 0x05, 0x4b, 0x68,
    0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a,
    0x10, 0xb2, 0x3d, 0xaf, 0x35, 0xcc, 0x90, 0xfa,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
#endif

static char pb_rm_interfaces__msg__RobotStatus__TYPE_NAME[] = "pb_rm_interfaces/msg/RobotStatus";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";

// Define type names, field names, and default values
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__robot_id[] = "robot_id";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__robot_level[] = "robot_level";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__current_hp[] = "current_hp";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__maximum_hp[] = "maximum_hp";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__shooter_barrel_cooling_value[] = "shooter_barrel_cooling_value";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__shooter_barrel_heat_limit[] = "shooter_barrel_heat_limit";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__shooter_17mm_1_barrel_heat[] = "shooter_17mm_1_barrel_heat";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__robot_pos[] = "robot_pos";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__armor_id[] = "armor_id";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__hp_deduction_reason[] = "hp_deduction_reason";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__projectile_allowance_17mm[] = "projectile_allowance_17mm";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__remaining_gold_coin[] = "remaining_gold_coin";
static char pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__is_hp_deduced[] = "is_hp_deduced";

static rosidl_runtime_c__type_description__Field pb_rm_interfaces__msg__RobotStatus__FIELDS[] = {
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__robot_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__robot_level, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__current_hp, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__maximum_hp, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__shooter_barrel_cooling_value, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__shooter_barrel_heat_limit, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__shooter_17mm_1_barrel_heat, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__robot_pos, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__armor_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__hp_deduction_reason, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__projectile_allowance_17mm, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__remaining_gold_coin, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RobotStatus__FIELD_NAME__is_hp_deduced, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription pb_rm_interfaces__msg__RobotStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pb_rm_interfaces__msg__RobotStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pb_rm_interfaces__msg__RobotStatus__TYPE_NAME, 32, 32},
      {pb_rm_interfaces__msg__RobotStatus__FIELDS, 13, 13},
    },
    {pb_rm_interfaces__msg__RobotStatus__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe6\\x80\\xa7\\xe8\\x83\\xbd\\xe4\\xbd\\x93\\xe7\\xb3\\xbb\\xe6\\x95\\xb0\\xe6\\x8d\\xae (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x0201)\n"
  "uint8 robot_id                          # \\xe6\\x9c\\xac\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba ID\n"
  "uint8 robot_level                       # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe7\\xad\\x89\\xe7\\xba\\xa7\n"
  "uint16 current_hp                       # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe5\\xbd\\x93\\xe5\\x89\\x8d\\xe8\\xa1\\x80\\xe9\\x87\\x8f\n"
  "uint16 maximum_hp                       # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe8\\xa1\\x80\\xe9\\x87\\x8f\\xe4\\xb8\\x8a\\xe9\\x99\\x90\n"
  "uint16 shooter_barrel_cooling_value     # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe6\\x9e\\xaa\\xe5\\x8f\\xa3\\xe7\\x83\\xad\\xe9\\x87\\x8f\\xe6\\xaf\\x8f\\xe7\\xa7\\x92\\xe5\\x86\\xb7\\xe5\\x8d\\xb4\\xe5\\x80\\xbc\n"
  "uint16 shooter_barrel_heat_limit        # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe6\\x9e\\xaa\\xe5\\x8f\\xa3\\xe7\\x83\\xad\\xe9\\x87\\x8f\\xe4\\xb8\\x8a\\xe9\\x99\\x90\n"
  "\n"
  "# \\xe5\\xae\\x9e\\xe6\\x97\\xb6\\xe5\\xba\\x95\\xe7\\x9b\\x98\\xe7\\xbc\\x93\\xe5\\x86\\xb2\\xe8\\x83\\xbd\\xe9\\x87\\x8f\\xe5\\x92\\x8c\\xe5\\xb0\\x84\\xe5\\x87\\xbb\\xe7\\x83\\xad\\xe9\\x87\\x8f\\xe6\\x95\\xb0 (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x0202)\n"
  "uint16 shooter_17mm_1_barrel_heat       # \\xe7\\xac\\xac 1 \\xe4\\xb8\\xaa 17mm \\xe5\\x8f\\x91\\xe5\\xb0\\x84\\xe6\\x9c\\xba\\xe6\\x9e\\x84\\xe7\\x9a\\x84\\xe6\\x9e\\xaa\\xe5\\x8f\\xa3\\xe7\\x83\\xad\\xe9\\x87\\x8f\n"
  "\n"
  "# \\xe6\\x9c\\xac\\xe6\\x9c\\xba\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe4\\xbd\\x8d\\xe7\\xbd\\xae\\xe6\\x95\\xb0\\xe6\\x8d\\xae (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x0203)\n"
  "geometry_msgs/Pose robot_pos              # \\xe6\\x9c\\xac\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe5\\xa7\\xbf\\xe6\\x80\\x81\n"
  "\n"
  "# \\xe4\\xbc\\xa4\\xe5\\xae\\xb3\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xe6\\x95\\xb0\\xe6\\x8d\\xae (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x0206)\n"
  "# const for hp_deduction_reason\n"
  "uint8 ARMOR_HIT = 0                     # \\xe8\\xa3\\x85\\xe7\\x94\\xb2\\xe6\\xa8\\xa1\\xe5\\x9d\\x97\\xe8\\xa2\\xab\\xe5\\xbc\\xb9\\xe4\\xb8\\xb8\\xe6\\x94\\xbb\\xe5\\x87\\xbb\\xe5\\xaf\\xbc\\xe8\\x87\\xb4\\xe6\\x89\\xa3\\xe8\\xa1\\x80\n"
  "uint8 SYSTEM_OFFLINE = 1                # \\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe9\\x87\\x8d\\xe8\\xa6\\x81\\xe6\\xa8\\xa1\\xe5\\x9d\\x97\\xe7\\xa6\\xbb\\xe7\\xba\\xbf\\xe5\\xaf\\xbc\\xe8\\x87\\xb4\\xe6\\x89\\xa3\\xe8\\xa1\\x80\n"
  "uint8 OVER_SHOOT_SPEED = 2              # \\xe5\\xb0\\x84\\xe5\\x87\\xbb\\xe5\\x88\\x9d\\xe9\\x80\\x9f\\xe5\\xba\\xa6\\xe8\\xb6\\x85\\xe9\\x99\\x90\\xe5\\xaf\\xbc\\xe8\\x87\\xb4\\xe6\\x89\\xa3\\xe8\\xa1\\x80\n"
  "uint8 OVER_HEAT = 3                     # \\xe6\\x9e\\xaa\\xe5\\x8f\\xa3\\xe7\\x83\\xad\\xe9\\x87\\x8f\\xe8\\xb6\\x85\\xe9\\x99\\x90\\xe5\\xaf\\xbc\\xe8\\x87\\xb4\\xe6\\x89\\xa3\\xe8\\xa1\\x80\n"
  "uint8 OVER_POWER = 4                    # \\xe5\\xba\\x95\\xe7\\x9b\\x98\\xe5\\x8a\\x9f\\xe7\\x8e\\x87\\xe8\\xb6\\x85\\xe9\\x99\\x90\\xe5\\xaf\\xbc\\xe8\\x87\\xb4\\xe6\\x89\\xa3\\xe8\\xa1\\x80\n"
  "uint8 ARMOR_COLLISION = 5               # \\xe8\\xa3\\x85\\xe7\\x94\\xb2\\xe6\\xa8\\xa1\\xe5\\x9d\\x97\\xe5\\x8f\\x97\\xe5\\x88\\xb0\\xe6\\x92\\x9e\\xe5\\x87\\xbb\\xe5\\xaf\\xbc\\xe8\\x87\\xb4\\xe6\\x89\\xa3\\xe8\\xa1\\x80\n"
  "\n"
  "uint8 armor_id                          # \\xe5\\xbd\\x93\\xe6\\x89\\xa3\\xe8\\xa1\\x80\\xe5\\x8e\\x9f\\xe5\\x9b\\xa0\\xe4\\xb8\\xba\\xe8\\xa3\\x85\\xe7\\x94\\xb2\\xe6\\xa8\\xa1\\xe5\\x9d\\x97\\xe8\\xa2\\xab\\xe5\\xbc\\xb9\\xe4\\xb8\\xb8\\xe6\\x94\\xbb\\xe5\\x87\\xbb\\xe3\\x80\\x81\\xe5\\x8f\\x97\\xe6\\x92\\x9e\\xe5\\x87\\xbb\\xe3\\x80\\x81\\xe7\\xa6\\xbb\\xe7\\xba\\xbf\\xe6\\x88\\x96\\xe6\\xb5\\x8b\\xe9\\x80\\x9f\\xe6\\xa8\\xa1\\xe5\\x9d\\x97\\xe7\\xa6\\xbb\\xe7\\xba\\xbf\\xe6\\x97\\xb6\\xef\\xbc\\x8c\n"
  "                                        # \\xe6\\x95\\xb0\\xe5\\x80\\xbc\\xe4\\xb8\\xba\\xe8\\xa3\\x85\\xe7\\x94\\xb2\\xe6\\xa8\\xa1\\xe5\\x9d\\x97\\xe6\\x88\\x96\\xe6\\xb5\\x8b\\xe9\\x80\\x9f\\xe6\\xa8\\xa1\\xe5\\x9d\\x97\\xe7\\x9a\\x84 ID \\xe7\\xbc\\x96\\xe5\\x8f\\xb7\\xef\\xbc\\x9b\\xe5\\xbd\\x93\\xe5\\x85\\xb6\\xe4\\xbb\\x96\\xe5\\x8e\\x9f\\xe5\\x9b\\xa0\\xe5\\xaf\\xbc\\xe8\\x87\\xb4\\xe6\\x89\\xa3\\xe8\\xa1\\x80\\xe6\\x97\\xb6\\xef\\xbc\\x8c\\xe8\\xaf\\xa5\\xe6\\x95\\xb0\\xe5\\x80\\xbc\\xe4\\xb8\\xba 0\n"
  "uint8 hp_deduction_reason               # \\xe8\\xa1\\x80\\xe9\\x87\\x8f\\xe5\\x8f\\x98\\xe5\\x8c\\x96\\xe7\\xb1\\xbb\\xe5\\x9e\\x8b\n"
  "\n"
  "# \\xe5\\x85\\x81\\xe8\\xae\\xb8\\xe5\\x8f\\x91\\xe5\\xbc\\xb9\\xe9\\x87\\x8f (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x0208)\n"
  "uint16 projectile_allowance_17mm        # 17mm \\xe5\\xbc\\xb9\\xe4\\xb8\\xb8\\xe5\\x89\\xa9\\xe4\\xbd\\x99\\xe5\\x8f\\x91\\xe5\\xb0\\x84\\xe6\\xac\\xa1\\xe6\\x95\\xb0\n"
  "uint16 remaining_gold_coin              # \\xe5\\x89\\xa9\\xe4\\xbd\\x99\\xe9\\x87\\x91\\xe5\\xb8\\x81\\xe6\\x95\\xb0\\xe9\\x87\\x8f\n"
  "\n"
  "bool is_hp_deduced                      # \\xe8\\xa1\\x80\\xe9\\x87\\x8f\\xe6\\x98\\xaf\\xe5\\x90\\xa6\\xe4\\xb8\\x8b\\xe9\\x99\\x8d\\xef\\xbc\\x88\\xe4\\xb8\\x8a\\xe4\\xbd\\x8d\\xe6\\x9c\\xba\\xe4\\xba\\x8c\\xe6\\xac\\xa1\\xe5\\xa4\\x84\\xe7\\x90\\x86\\xef\\xbc\\x89";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pb_rm_interfaces__msg__RobotStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pb_rm_interfaces__msg__RobotStatus__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1334, 1334},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pb_rm_interfaces__msg__RobotStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pb_rm_interfaces__msg__RobotStatus__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
