// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pb_rm_interfaces:msg/Models.idl
// generated code does not contain a copyright notice

#include "pb_rm_interfaces/msg/detail/models__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pb_rm_interfaces
const rosidl_type_hash_t *
pb_rm_interfaces__msg__Models__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf8, 0xc3, 0xbc, 0x19, 0xee, 0x70, 0xf0, 0xad,
      0xe8, 0xcf, 0x9c, 0x0b, 0x98, 0x2c, 0xef, 0xe6,
      0xe8, 0xae, 0xb0, 0xb7, 0xb0, 0x33, 0x83, 0x86,
      0x8d, 0x55, 0x2c, 0x8b, 0xd4, 0xac, 0x04, 0x8e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char pb_rm_interfaces__msg__Models__TYPE_NAME[] = "pb_rm_interfaces/msg/Models";

// Define type names, field names, and default values
static char pb_rm_interfaces__msg__Models__FIELD_NAME__chassis[] = "chassis";
static char pb_rm_interfaces__msg__Models__FIELD_NAME__gimbal[] = "gimbal";
static char pb_rm_interfaces__msg__Models__FIELD_NAME__shoot[] = "shoot";
static char pb_rm_interfaces__msg__Models__FIELD_NAME__arm[] = "arm";
static char pb_rm_interfaces__msg__Models__FIELD_NAME__custom_controller[] = "custom_controller";

static rosidl_runtime_c__type_description__Field pb_rm_interfaces__msg__Models__FIELDS[] = {
  {
    {pb_rm_interfaces__msg__Models__FIELD_NAME__chassis, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Models__FIELD_NAME__gimbal, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Models__FIELD_NAME__shoot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Models__FIELD_NAME__arm, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Models__FIELD_NAME__custom_controller, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pb_rm_interfaces__msg__Models__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pb_rm_interfaces__msg__Models__TYPE_NAME, 27, 27},
      {pb_rm_interfaces__msg__Models__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string chassis\n"
  "string gimbal\n"
  "string shoot\n"
  "string arm\n"
  "string custom_controller";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pb_rm_interfaces__msg__Models__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pb_rm_interfaces__msg__Models__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 77, 77},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pb_rm_interfaces__msg__Models__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pb_rm_interfaces__msg__Models__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
