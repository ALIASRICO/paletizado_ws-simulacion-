// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dobot_msgs_v4:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#include "dobot_msgs_v4/msg/detail/robot_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__msg__RobotStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xff, 0x49, 0x35, 0x53, 0xe1, 0xd2, 0xb7, 0xa3,
      0xef, 0x36, 0xf7, 0x34, 0xf4, 0xd4, 0x22, 0xac,
      0xd3, 0x89, 0x75, 0xe3, 0x0d, 0xc7, 0x0d, 0xae,
      0xb1, 0x05, 0x47, 0x82, 0x73, 0x41, 0x80, 0x1b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char dobot_msgs_v4__msg__RobotStatus__TYPE_NAME[] = "dobot_msgs_v4/msg/RobotStatus";

// Define type names, field names, and default values
static char dobot_msgs_v4__msg__RobotStatus__FIELD_NAME__is_enable[] = "is_enable";
static char dobot_msgs_v4__msg__RobotStatus__FIELD_NAME__is_connected[] = "is_connected";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__msg__RobotStatus__FIELDS[] = {
  {
    {dobot_msgs_v4__msg__RobotStatus__FIELD_NAME__is_enable, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__msg__RobotStatus__FIELD_NAME__is_connected, 12, 12},
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
dobot_msgs_v4__msg__RobotStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__msg__RobotStatus__TYPE_NAME, 29, 29},
      {dobot_msgs_v4__msg__RobotStatus__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool is_enable\n"
  "bool is_connected";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__msg__RobotStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__msg__RobotStatus__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 32, 32},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__msg__RobotStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__msg__RobotStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
