// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pb_rm_interfaces:msg/GameStatus.idl
// generated code does not contain a copyright notice

#include "pb_rm_interfaces/msg/detail/game_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pb_rm_interfaces
const rosidl_type_hash_t *
pb_rm_interfaces__msg__GameStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5f, 0xd8, 0xf1, 0x0e, 0xba, 0x25, 0x15, 0x15,
      0x44, 0xea, 0x69, 0xfb, 0xf8, 0x71, 0x7c, 0xa2,
      0xd8, 0xe0, 0x64, 0x3c, 0x31, 0x24, 0xb6, 0xfd,
      0x5d, 0x48, 0x27, 0xe8, 0x64, 0x97, 0x45, 0xb2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char pb_rm_interfaces__msg__GameStatus__TYPE_NAME[] = "pb_rm_interfaces/msg/GameStatus";

// Define type names, field names, and default values
static char pb_rm_interfaces__msg__GameStatus__FIELD_NAME__game_type[] = "game_type";
static char pb_rm_interfaces__msg__GameStatus__FIELD_NAME__game_progress[] = "game_progress";
static char pb_rm_interfaces__msg__GameStatus__FIELD_NAME__stage_remain_time[] = "stage_remain_time";

static rosidl_runtime_c__type_description__Field pb_rm_interfaces__msg__GameStatus__FIELDS[] = {
  {
    {pb_rm_interfaces__msg__GameStatus__FIELD_NAME__game_type, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__GameStatus__FIELD_NAME__game_progress, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__GameStatus__FIELD_NAME__stage_remain_time, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pb_rm_interfaces__msg__GameStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pb_rm_interfaces__msg__GameStatus__TYPE_NAME, 31, 31},
      {pb_rm_interfaces__msg__GameStatus__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe6\\xaf\\x94\\xe8\\xb5\\x9b\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xe6\\x95\\xb0\\xe6\\x8d\\xae (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x0001)\n"
  "\n"
  "# constants for game progress\n"
  "uint8 NOT_START = 0                 # \\xe6\\x9c\\xaa\\xe5\\xbc\\x80\\xe5\\xa7\\x8b\\xe6\\xaf\\x94\\xe8\\xb5\\x9b\n"
  "uint8 PREPARATION = 1               # \\xe5\\x87\\x86\\xe5\\xa4\\x87\\xe9\\x98\\xb6\\xe6\\xae\\xb5\n"
  "uint8 SELF_CHECKING = 2             # \\xe5\\x8d\\x81\\xe4\\xba\\x94\\xe7\\xa7\\x92\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe8\\x87\\xaa\\xe6\\xa3\\x80\\xe9\\x98\\xb6\\xe6\\xae\\xb5\n"
  "uint8 COUNT_DOWN = 3                # \\xe4\\xba\\x94\\xe7\\xa7\\x92\\xe5\\x80\\x92\\xe8\\xae\\xa1\\xe6\\x97\\xb6\n"
  "uint8 RUNNING = 4                   # \\xe6\\xaf\\x94\\xe8\\xb5\\x9b\\xe4\\xb8\\xad\n"
  "uint8 GAME_OVER = 5                 # \\xe6\\xaf\\x94\\xe8\\xb5\\x9b\\xe7\\xbb\\x93\\xe7\\xae\\x97\\xe4\\xb8\\xad\n"
  "\n"
  "\n"
  "uint8 game_type\n"
  "uint8 game_progress\n"
  "int32 stage_remain_time             # \\xe5\\xbd\\x93\\xe5\\x89\\x8d\\xe9\\x98\\xb6\\xe6\\xae\\xb5\\xe5\\x89\\xa9\\xe4\\xbd\\x99\\xe6\\x97\\xb6\\xe9\\x97\\xb4\\xef\\xbc\\x8c\\xe5\\x8d\\x95\\xe4\\xbd\\x8d\\xef\\xbc\\x9a\\xe7\\xa7\\x92";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pb_rm_interfaces__msg__GameStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pb_rm_interfaces__msg__GameStatus__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 422, 422},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pb_rm_interfaces__msg__GameStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pb_rm_interfaces__msg__GameStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
