// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pb_rm_interfaces:msg/EventData.idl
// generated code does not contain a copyright notice

#include "pb_rm_interfaces/msg/detail/event_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pb_rm_interfaces
const rosidl_type_hash_t *
pb_rm_interfaces__msg__EventData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbb, 0x17, 0x49, 0x5d, 0xbd, 0x2b, 0x4c, 0x4b,
      0x14, 0x98, 0xee, 0x28, 0x83, 0x56, 0x00, 0x46,
      0xae, 0xb7, 0xbc, 0x94, 0x0b, 0xbb, 0x2f, 0xab,
      0x93, 0x58, 0x7c, 0x26, 0x0f, 0x4a, 0x3e, 0x34,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char pb_rm_interfaces__msg__EventData__TYPE_NAME[] = "pb_rm_interfaces/msg/EventData";

// Define type names, field names, and default values
static char pb_rm_interfaces__msg__EventData__FIELD_NAME__non_overlapping_supply_zone[] = "non_overlapping_supply_zone";
static char pb_rm_interfaces__msg__EventData__FIELD_NAME__overlapping_supply_zone[] = "overlapping_supply_zone";
static char pb_rm_interfaces__msg__EventData__FIELD_NAME__supply_zone[] = "supply_zone";
static char pb_rm_interfaces__msg__EventData__FIELD_NAME__small_energy[] = "small_energy";
static char pb_rm_interfaces__msg__EventData__FIELD_NAME__big_energy[] = "big_energy";
static char pb_rm_interfaces__msg__EventData__FIELD_NAME__central_highland[] = "central_highland";
static char pb_rm_interfaces__msg__EventData__FIELD_NAME__trapezoidal_highland[] = "trapezoidal_highland";
static char pb_rm_interfaces__msg__EventData__FIELD_NAME__center_gain_zone[] = "center_gain_zone";

static rosidl_runtime_c__type_description__Field pb_rm_interfaces__msg__EventData__FIELDS[] = {
  {
    {pb_rm_interfaces__msg__EventData__FIELD_NAME__non_overlapping_supply_zone, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__EventData__FIELD_NAME__overlapping_supply_zone, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__EventData__FIELD_NAME__supply_zone, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__EventData__FIELD_NAME__small_energy, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__EventData__FIELD_NAME__big_energy, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__EventData__FIELD_NAME__central_highland, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__EventData__FIELD_NAME__trapezoidal_highland, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__EventData__FIELD_NAME__center_gain_zone, 16, 16},
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
pb_rm_interfaces__msg__EventData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pb_rm_interfaces__msg__EventData__TYPE_NAME, 30, 30},
      {pb_rm_interfaces__msg__EventData__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe5\\x9c\\xba\\xe5\\x9c\\xb0\\xe4\\xba\\x8b\\xe4\\xbb\\xb6\\xe6\\x95\\xb0\\xe6\\x8d\\xae (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x0101)\n"
  "\n"
  "# Constants for the occupation and activation states\n"
  "uint8 UNOCCUPIED = 0                # Not occupied or not activated\n"
  "uint8 OCCUPIED_FRIEND = 1           # Occupied or activated by friendly side\n"
  "uint8 OCCUPIED_ENEMY = 2            # Occupied or activated by enemy side\n"
  "uint8 OCCUPIED_BOTH = 3             # Occupied or activated by both sides\n"
  "\n"
  "uint8 non_overlapping_supply_zone   # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe4\\xb8\\x8e\\xe5\\x85\\x91\\xe6\\x8d\\xa2\\xe5\\x8c\\xba\\xe4\\xb8\\x8d\\xe9\\x87\\x8d\\xe5\\x8f\\xa0\\xe7\\x9a\\x84\\xe8\\xa1\\xa5\\xe7\\xbb\\x99\\xe5\\x8c\\xba\\xe7\\x9a\\x84\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xef\\xbc\\x8c1 \\xe4\\xb8\\xba\\xe5\\xb7\\xb2\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\n"
  "uint8 overlapping_supply_zone       # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe4\\xb8\\x8e\\xe5\\x85\\x91\\xe6\\x8d\\xa2\\xe5\\x8c\\xba\\xe9\\x87\\x8d\\xe5\\x8f\\xa0\\xe7\\x9a\\x84\\xe8\\xa1\\xa5\\xe7\\xbb\\x99\\xe5\\x8c\\xba\\xe7\\x9a\\x84\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xef\\xbc\\x8c1 \\xe4\\xb8\\xba\\xe5\\xb7\\xb2\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\n"
  "uint8 supply_zone                   # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe8\\xa1\\xa5\\xe7\\xbb\\x99\\xe5\\x8c\\xba\\xe7\\x9a\\x84\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xef\\xbc\\x8c1 \\xe4\\xb8\\xba\\xe5\\xb7\\xb2\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xef\\xbc\\x88\\xe4\\xbb\\x85 RMUL \\xe9\\x80\\x82\\xe7\\x94\\xa8\\xef\\xbc\\x89\n"
  "\n"
  "uint8 small_energy                  # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\xb0\\x8f\\xe8\\x83\\xbd\\xe9\\x87\\x8f\\xe6\\x9c\\xba\\xe5\\x85\\xb3\\xe7\\x9a\\x84\\xe6\\xbf\\x80\\xe6\\xb4\\xbb\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xef\\xbc\\x8c1 \\xe4\\xb8\\xba\\xe5\\xb7\\xb2\\xe6\\xbf\\x80\\xe6\\xb4\\xbb\n"
  "uint8 big_energy                    # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\xa4\\xa7\\xe8\\x83\\xbd\\xe9\\x87\\x8f\\xe6\\x9c\\xba\\xe5\\x85\\xb3\\xe7\\x9a\\x84\\xe6\\xbf\\x80\\xe6\\xb4\\xbb\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xef\\xbc\\x8c1 \\xe4\\xb8\\xba\\xe5\\xb7\\xb2\\xe6\\xbf\\x80\\xe6\\xb4\\xbb\n"
  "\n"
  "uint8 central_highland              # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe4\\xb8\\xad\\xe5\\xa4\\xae\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe7\\x9a\\x84\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xef\\xbc\\x8c1 \\xe4\\xb8\\xba\\xe8\\xa2\\xab\\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xef\\xbc\\x8c2 \\xe4\\xb8\\xba\\xe8\\xa2\\xab\\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\n"
  "uint8 trapezoidal_highland          # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe6\\xa2\\xaf\\xe5\\xbd\\xa2\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe7\\x9a\\x84\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xe7\\x8a\\xb6\\xe6\\x80\\x81\\xef\\xbc\\x8c1 \\xe4\\xb8\\xba\\xe8\\xa2\\xab\\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xef\\xbc\\x8c2 \\xe4\\xb8\\xba\\xe8\\xa2\\xab\\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\n"
  "\n"
  "uint8 center_gain_zone              # \\xe4\\xb8\\xad\\xe5\\xbf\\x83\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xe7\\x9a\\x84\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xe6\\x83\\x85\\xe5\\x86\\xb5\\xef\\xbc\\x8c\n"
  "                                    # 0 \\xe4\\xb8\\xba\\xe6\\x9c\\xaa\\xe8\\xa2\\xab\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xef\\xbc\\x8c1 \\xe4\\xb8\\xba\\xe8\\xa2\\xab\\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xef\\xbc\\x8c2 \\xe4\\xb8\\xba\\xe8\\xa2\\xab\\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xef\\xbc\\x8c3 \\xe4\\xb8\\xba\\xe8\\xa2\\xab\\xe5\\x8f\\x8c\\xe6\\x96\\xb9\\xe5\\x8d\\xa0\\xe9\\xa2\\x86\\xef\\xbc\\x88\\xe4\\xbb\\x85 RMUL \\xe9\\x80\\x82\\xe7\\x94\\xa8\\xef\\xbc\\x89";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pb_rm_interfaces__msg__EventData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pb_rm_interfaces__msg__EventData__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 965, 965},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pb_rm_interfaces__msg__EventData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pb_rm_interfaces__msg__EventData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
