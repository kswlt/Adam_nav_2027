// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from pb_rm_interfaces:msg/RfidStatus.idl
// generated code does not contain a copyright notice

#include "pb_rm_interfaces/msg/detail/rfid_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_pb_rm_interfaces
const rosidl_type_hash_t *
pb_rm_interfaces__msg__RfidStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5b, 0x41, 0x7f, 0x88, 0xc9, 0xa8, 0x69, 0xd5,
      0x8f, 0xc2, 0xbe, 0x65, 0x52, 0x45, 0xce, 0x96,
      0x18, 0x39, 0x34, 0xd7, 0xdb, 0x55, 0x12, 0x81,
      0x1a, 0x15, 0x59, 0x31, 0x9e, 0x05, 0x20, 0x69,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char pb_rm_interfaces__msg__RfidStatus__TYPE_NAME[] = "pb_rm_interfaces/msg/RfidStatus";

// Define type names, field names, and default values
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__base_gain_point[] = "base_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__central_highland_gain_point[] = "central_highland_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_central_highland_gain_point[] = "enemy_central_highland_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_trapezoidal_highland_gain_point[] = "friendly_trapezoidal_highland_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_trapezoidal_highland_gain_point[] = "enemy_trapezoidal_highland_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_fly_ramp_front_gain_point[] = "friendly_fly_ramp_front_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_fly_ramp_back_gain_point[] = "friendly_fly_ramp_back_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_fly_ramp_front_gain_point[] = "enemy_fly_ramp_front_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_fly_ramp_back_gain_point[] = "enemy_fly_ramp_back_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_central_highland_lower_gain_point[] = "friendly_central_highland_lower_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_central_highland_upper_gain_point[] = "friendly_central_highland_upper_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_central_highland_lower_gain_point[] = "enemy_central_highland_lower_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_central_highland_upper_gain_point[] = "enemy_central_highland_upper_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_highway_lower_gain_point[] = "friendly_highway_lower_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_highway_upper_gain_point[] = "friendly_highway_upper_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_highway_lower_gain_point[] = "enemy_highway_lower_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_highway_upper_gain_point[] = "enemy_highway_upper_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_fortress_gain_point[] = "friendly_fortress_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_outpost_gain_point[] = "friendly_outpost_gain_point";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_supply_zone_non_exchange[] = "friendly_supply_zone_non_exchange";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_supply_zone_exchange[] = "friendly_supply_zone_exchange";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_big_resource_island[] = "friendly_big_resource_island";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_big_resource_island[] = "enemy_big_resource_island";
static char pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__center_gain_point[] = "center_gain_point";

static rosidl_runtime_c__type_description__Field pb_rm_interfaces__msg__RfidStatus__FIELDS[] = {
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__base_gain_point, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__central_highland_gain_point, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_central_highland_gain_point, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_trapezoidal_highland_gain_point, 40, 40},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_trapezoidal_highland_gain_point, 37, 37},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_fly_ramp_front_gain_point, 34, 34},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_fly_ramp_back_gain_point, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_fly_ramp_front_gain_point, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_fly_ramp_back_gain_point, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_central_highland_lower_gain_point, 42, 42},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_central_highland_upper_gain_point, 42, 42},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_central_highland_lower_gain_point, 39, 39},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_central_highland_upper_gain_point, 39, 39},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_highway_lower_gain_point, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_highway_upper_gain_point, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_highway_lower_gain_point, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_highway_upper_gain_point, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_fortress_gain_point, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_outpost_gain_point, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_supply_zone_non_exchange, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_supply_zone_exchange, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__friendly_big_resource_island, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__enemy_big_resource_island, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {pb_rm_interfaces__msg__RfidStatus__FIELD_NAME__center_gain_point, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
pb_rm_interfaces__msg__RfidStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {pb_rm_interfaces__msg__RfidStatus__TYPE_NAME, 31, 31},
      {pb_rm_interfaces__msg__RfidStatus__FIELDS, 24, 24},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xe6\\x9c\\xba\\xe5\\x99\\xa8\\xe4\\xba\\xba RFID \\xe6\\xa8\\xa1\\xe5\\x9d\\x97\\xe7\\x8a\\xb6\\xe6\\x80\\x81 (\\xe8\\xa3\\x81\\xe5\\x88\\xa4\\xe7\\xb3\\xbb\\xe7\\xbb\\x9f\\xe4\\xb8\\xb2\\xe5\\x8f\\xa3\\xe5\\x8d\\x8f\\xe8\\xae\\xae V1.7.0 0x0209)\n"
  "\n"
  "# const for RFID status\n"
  "uint8 NOT_DETECTED = 0                              # RFID card not detected\n"
  "uint8 DETECTED = 1                                  # RFID card detected\n"
  "\n"
  "bool base_gain_point                                # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x9f\\xba\\xe5\\x9c\\xb0\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\n"
  "bool central_highland_gain_point                    # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe4\\xb8\\xad\\xe5\\xa4\\xae\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\n"
  "bool enemy_central_highland_gain_point              # \\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe4\\xb8\\xad\\xe5\\xa4\\xae\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\n"
  "bool friendly_trapezoidal_highland_gain_point       # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe6\\xa2\\xaf\\xe5\\xbd\\xa2\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\n"
  "bool enemy_trapezoidal_highland_gain_point          # \\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe6\\xa2\\xaf\\xe5\\xbd\\xa2\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\n"
  "bool friendly_fly_ramp_front_gain_point             # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe9\\xa3\\x9e\\xe5\\x9d\\xa1\\xef\\xbc\\x89\\xef\\xbc\\x88\\xe9\\x9d\\xa0\\xe8\\xbf\\x91\\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe4\\xb8\\x80\\xe4\\xbe\\xa7\\xe9\\xa3\\x9e\\xe5\\x9d\\xa1\\xe5\\x89\\x8d\\xef\\xbc\\x89\n"
  "bool friendly_fly_ramp_back_gain_point              # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe9\\xa3\\x9e\\xe5\\x9d\\xa1\\xef\\xbc\\x89\\xef\\xbc\\x88\\xe9\\x9d\\xa0\\xe8\\xbf\\x91\\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe4\\xb8\\x80\\xe4\\xbe\\xa7\\xe9\\xa3\\x9e\\xe5\\x9d\\xa1\\xe5\\x90\\x8e\\xef\\xbc\\x89\n"
  "bool enemy_fly_ramp_front_gain_point                # \\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe9\\xa3\\x9e\\xe5\\x9d\\xa1\\xef\\xbc\\x89\\xef\\xbc\\x88\\xe9\\x9d\\xa0\\xe8\\xbf\\x91\\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe4\\xb8\\x80\\xe4\\xbe\\xa7\\xe9\\xa3\\x9e\\xe5\\x9d\\xa1\\xe5\\x89\\x8d\\xef\\xbc\\x89\n"
  "bool enemy_fly_ramp_back_gain_point                 # \\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe9\\xa3\\x9e\\xe5\\x9d\\xa1\\xef\\xbc\\x89\\xef\\xbc\\x88\\xe9\\x9d\\xa0\\xe8\\xbf\\x91\\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe4\\xb8\\x80\\xe4\\xbe\\xa7\\xe9\\xa3\\x9e\\xe5\\x9d\\xa1\\xe5\\x90\\x8e\\xef\\xbc\\x89\n"
  "bool friendly_central_highland_lower_gain_point     # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe4\\xb8\\xad\\xe5\\xa4\\xae\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe4\\xb8\\x8b\\xe6\\x96\\xb9\\xef\\xbc\\x89\n"
  "bool friendly_central_highland_upper_gain_point     # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe4\\xb8\\xad\\xe5\\xa4\\xae\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe4\\xb8\\x8a\\xe6\\x96\\xb9\\xef\\xbc\\x89\n"
  "bool enemy_central_highland_lower_gain_point        # \\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe4\\xb8\\xad\\xe5\\xa4\\xae\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe4\\xb8\\x8b\\xe6\\x96\\xb9\\xef\\xbc\\x89\n"
  "bool enemy_central_highland_upper_gain_point        # \\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe4\\xb8\\xad\\xe5\\xa4\\xae\\xe9\\xab\\x98\\xe5\\x9c\\xb0\\xe4\\xb8\\x8a\\xe6\\x96\\xb9\\xef\\xbc\\x89\n"
  "bool friendly_highway_lower_gain_point              # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe5\\x85\\xac\\xe8\\xb7\\xaf\\xe4\\xb8\\x8b\\xe6\\x96\\xb9\\xef\\xbc\\x89\n"
  "bool friendly_highway_upper_gain_point              # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe5\\x85\\xac\\xe8\\xb7\\xaf\\xe4\\xb8\\x8a\\xe6\\x96\\xb9\\xef\\xbc\\x89\n"
  "bool enemy_highway_lower_gain_point                 # \\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe5\\x85\\xac\\xe8\\xb7\\xaf\\xe4\\xb8\\x8b\\xe6\\x96\\xb9\\xef\\xbc\\x89\n"
  "bool enemy_highway_upper_gain_point                 # \\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\x9c\\xb0\\xe5\\xbd\\xa2\\xe8\\xb7\\xa8\\xe8\\xb6\\x8a\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe5\\x85\\xac\\xe8\\xb7\\xaf\\xe4\\xb8\\x8a\\xe6\\x96\\xb9\\xef\\xbc\\x89\n"
  "bool friendly_fortress_gain_point                   # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\xa0\\xa1\\xe5\\x9e\\x92\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\n"
  "bool friendly_outpost_gain_point                    # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\x89\\x8d\\xe5\\x93\\xa8\\xe7\\xab\\x99\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\n"
  "bool friendly_supply_zone_non_exchange              # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe4\\xb8\\x8e\\xe5\\x85\\x91\\xe6\\x8d\\xa2\\xe5\\x8c\\xba\\xe4\\xb8\\x8d\\xe9\\x87\\x8d\\xe5\\x8f\\xa0\\xe7\\x9a\\x84\\xe8\\xa1\\xa5\\xe7\\xbb\\x99\\xe5\\x8c\\xba/RMUL \\xe8\\xa1\\xa5\\xe7\\xbb\\x99\\xe5\\x8c\\xba\n"
  "bool friendly_supply_zone_exchange                  # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe4\\xb8\\x8e\\xe5\\x85\\x91\\xe6\\x8d\\xa2\\xe5\\x8c\\xba\\xe9\\x87\\x8d\\xe5\\x8f\\xa0\\xe7\\x9a\\x84\\xe8\\xa1\\xa5\\xe7\\xbb\\x99\\xe5\\x8c\\xba\n"
  "bool friendly_big_resource_island                   # \\xe5\\xb7\\xb1\\xe6\\x96\\xb9\\xe5\\xa4\\xa7\\xe8\\xb5\\x84\\xe6\\xba\\x90\\xe5\\xb2\\x9b\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\n"
  "bool enemy_big_resource_island                      # \\xe5\\xaf\\xb9\\xe6\\x96\\xb9\\xe5\\xa4\\xa7\\xe8\\xb5\\x84\\xe6\\xba\\x90\\xe5\\xb2\\x9b\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\n"
  "bool center_gain_point                              # \\xe4\\xb8\\xad\\xe5\\xbf\\x83\\xe5\\xa2\\x9e\\xe7\\x9b\\x8a\\xe7\\x82\\xb9\\xef\\xbc\\x88\\xe4\\xbb\\x85 RMUL \\xe9\\x80\\x82\\xe7\\x94\\xa8\\xef\\xbc\\x89";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
pb_rm_interfaces__msg__RfidStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {pb_rm_interfaces__msg__RfidStatus__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1887, 1887},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
pb_rm_interfaces__msg__RfidStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *pb_rm_interfaces__msg__RfidStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
