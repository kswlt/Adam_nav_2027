// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pb_rm_interfaces:msg/Buff.idl
// generated code does not contain a copyright notice

#include "pb_rm_interfaces/msg/detail/buff__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pb_rm_interfaces
const rosidl_type_hash_t *
pb_rm_interfaces__msg__Buff__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x23, 0x33, 0x95, 0xde, 0x5a, 0x40, 0x69, 0xc1,
      0xae, 0xe9, 0x95, 0xbc, 0xc7, 0x9b, 0xf7, 0x70,
      0xb9, 0x51, 0x95, 0xfe, 0x90, 0x2d, 0x98, 0xa4,
      0x02, 0x41, 0xf7, 0xbd, 0xd8, 0xa9, 0x09, 0x39,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char pb_rm_interfaces__msg__Buff__TYPE_NAME[] = "pb_rm_interfaces/msg/Buff";

// Define type names, field names, and default values
static char pb_rm_interfaces__msg__Buff__FIELD_NAME__recovery_buff[] = "recovery_buff";
static char pb_rm_interfaces__msg__Buff__FIELD_NAME__cooling_buff[] = "cooling_buff";
static char pb_rm_interfaces__msg__Buff__FIELD_NAME__defence_buff[] = "defence_buff";
static char pb_rm_interfaces__msg__Buff__FIELD_NAME__vulnerability_buff[] = "vulnerability_buff";
static char pb_rm_interfaces__msg__Buff__FIELD_NAME__attack_buff[] = "attack_buff";
static char pb_rm_interfaces__msg__Buff__FIELD_NAME__remaining_energy[] = "remaining_energy";

static rosidl_runtime_c__type_description__Field pb_rm_interfaces__msg__Buff__FIELDS[] = {
  {
    {pb_rm_interfaces__msg__Buff__FIELD_NAME__recovery_buff, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Buff__FIELD_NAME__cooling_buff, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Buff__FIELD_NAME__defence_buff, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Buff__FIELD_NAME__vulnerability_buff, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Buff__FIELD_NAME__attack_buff, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__Buff__FIELD_NAME__remaining_energy, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pb_rm_interfaces__msg__Buff__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pb_rm_interfaces__msg__Buff__TYPE_NAME, 25, 25},
      {pb_rm_interfaces__msg__Buff__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe5\\x92\\x8c\\xe5\\xba\\x95\\xe7\\x9b\\x98\\xe8\\x83\\xbd\\xe9\\x87\\x8f\\xe6\\x95\\xb0\\xe6\\x8d\\xae (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x0204)\n"
  "\n"
  "uint8 recovery_buff           # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe5\\x9b\\x9e\\xe8\\xa1\\x80\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xef\\xbc\\x88\\xe7\\x99\\xbe\\xe5\\x88\\x86\\xe6\\xaf\\x94\\xef\\xbc\\x8c\\xe5\\x80\\xbc\\xe4\\xb8\\xba 10 \\xe8\\xa1\\xa8\\xe7\\xa4\\xba\\xe6\\xaf\\x8f\\xe7\\xa7\\x92\\xe6\\x81\\xa2\\xe5\\xa4\\x8d\\xe8\\xa1\\x80\\xe9\\x87\\x8f\\xe4\\xb8\\x8a\\xe9\\x99\\x90\\xe7\\x9a\\x84 10%\\xef\\xbc\\x89\n"
  "uint8 cooling_buff            # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe5\\xb0\\x84\\xe5\\x87\\xbb\\xe7\\x83\\xad\\xe9\\x87\\x8f\\xe5\\x86\\xb7\\xe5\\x8d\\xb4\\xe5\\x80\\x8d\\xe7\\x8e\\x87\\xef\\xbc\\x88\\xe7\\x9b\\xb4\\xe6\\x8e\\xa5\\xe5\\x80\\xbc\\xef\\xbc\\x8c\\xe5\\x80\\xbc\\xe4\\xb8\\xba 5 \\xe8\\xa1\\xa8\\xe7\\xa4\\xba 5 \\xe5\\x80\\x8d\\xe5\\x86\\xb7\\xe5\\x8d\\xb4\\xef\\xbc\\x89\n"
  "uint8 defence_buff            # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe9\\x98\\xb2\\xe5\\xbe\\xa1\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xef\\xbc\\x88\\xe7\\x99\\xbe\\xe5\\x88\\x86\\xe6\\xaf\\x94\\xef\\xbc\\x8c\\xe5\\x80\\xbc\\xe4\\xb8\\xba 50 \\xe8\\xa1\\xa8\\xe7\\xa4\\xba 50% \\xe9\\x98\\xb2\\xe5\\xbe\\xa1\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xef\\xbc\\x89\n"
  "uint8 vulnerability_buff      # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe8\\xb4\\x9f\\xe9\\x98\\xb2\\xe5\\xbe\\xa1\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xef\\xbc\\x88\\xe7\\x99\\xbe\\xe5\\x88\\x86\\xe6\\xaf\\x94\\xef\\xbc\\x8c\\xe5\\x80\\xbc\\xe4\\xb8\\xba 30 \\xe8\\xa1\\xa8\\xe7\\xa4\\xba -30% \\xe9\\x98\\xb2\\xe5\\xbe\\xa1\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xef\\xbc\\x89\n"
  "uint16 attack_buff            # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe6\\x94\\xbb\\xe5\\x87\\xbb\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xef\\xbc\\x88\\xe7\\x99\\xbe\\xe5\\x88\\x86\\xe6\\xaf\\x94\\xef\\xbc\\x8c\\xe5\\x80\\xbc\\xe4\\xb8\\xba 50 \\xe8\\xa1\\xa8\\xe7\\xa4\\xba 50% \\xe6\\x94\\xbb\\xe5\\x87\\xbb\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xef\\xbc\\x89\n"
  "uint8 remaining_energy        # \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe5\\x89\\xa9\\xe4\\xbd\\x99\\xe8\\x83\\xbd\\xe9\\x87\\x8f\\xe5\\x80\\xbc\\xe5\\x8f\\x8d\\xe9\\xa6\\x88\\xef\\xbc\\x8c\\xe4\\xbb\\xa5 16 \\xe8\\xbf\\x9b\\xe5\\x88\\xb6\\xe6\\xa0\\x87\\xe8\\xaf\\x86\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe5\\x89\\xa9\\xe4\\xbd\\x99\\xe8\\x83\\xbd\\xe9\\x87\\x8f\\xe5\\x80\\xbc\\xe6\\xaf\\x94\\xe4\\xbe\\x8b\\xef\\xbc\\x8c\\xe4\\xbb\\x85\\xe5\\x9c\\xa8\\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba\\xe5\\x89\\xa9\\xe4\\xbd\\x99\\xe8\\x83\\xbd\\xe9\\x87\\x8f\\xe5\\xb0\\x8f\\xe4\\xba\\x8e 50% \\xe6\\x97\\xb6\\xe5\\x8f\\x8d\\xe9\\xa6\\x88\\xef\\xbc\\x8c\\xe5\\x85\\xb6\\xe4\\xbd\\x99\\xe9\\xbb\\x98\\xe8\\xae\\xa4\\xe5\\x8f\\x8d\\xe9\\xa6\\x88 0x32\\xe3\\x80\\x82";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pb_rm_interfaces__msg__Buff__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pb_rm_interfaces__msg__Buff__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 458, 458},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pb_rm_interfaces__msg__Buff__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pb_rm_interfaces__msg__Buff__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
