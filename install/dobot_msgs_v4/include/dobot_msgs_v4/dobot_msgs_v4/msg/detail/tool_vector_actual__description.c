// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dobot_msgs_v4:msg/ToolVectorActual.idl
// generated code does not contain a copyright notice

#include "dobot_msgs_v4/msg/detail/tool_vector_actual__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__msg__ToolVectorActual__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x97, 0xdd, 0x39, 0x4b, 0xe8, 0x6a, 0x15, 0x18,
      0x68, 0xfd, 0x1d, 0x1d, 0x37, 0xb3, 0x75, 0xf9,
      0x84, 0xf7, 0xc5, 0x66, 0x2e, 0x2d, 0x17, 0xcd,
      0x7f, 0xde, 0xd4, 0xe8, 0x7f, 0x0c, 0x1c, 0xfd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char dobot_msgs_v4__msg__ToolVectorActual__TYPE_NAME[] = "dobot_msgs_v4/msg/ToolVectorActual";

// Define type names, field names, and default values
static char dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__x[] = "x";
static char dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__y[] = "y";
static char dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__z[] = "z";
static char dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__rx[] = "rx";
static char dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__ry[] = "ry";
static char dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__rz[] = "rz";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__msg__ToolVectorActual__FIELDS[] = {
  {
    {dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__rx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__ry, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__msg__ToolVectorActual__FIELD_NAME__rz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__msg__ToolVectorActual__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__msg__ToolVectorActual__TYPE_NAME, 34, 34},
      {dobot_msgs_v4__msg__ToolVectorActual__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "float64 rx\n"
  "float64 ry\n"
  "float64 rz";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__msg__ToolVectorActual__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__msg__ToolVectorActual__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 62, 62},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__msg__ToolVectorActual__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__msg__ToolVectorActual__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
