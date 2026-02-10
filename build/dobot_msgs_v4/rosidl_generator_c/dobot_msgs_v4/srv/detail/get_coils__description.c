// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dobot_msgs_v4:srv/GetCoils.idl
// generated code does not contain a copyright notice

#include "dobot_msgs_v4/srv/detail/get_coils__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__GetCoils__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x33, 0xb5, 0x5e, 0xdd, 0xad, 0x06, 0x58, 0x8c,
      0x13, 0x8e, 0xd2, 0xe6, 0xbd, 0x18, 0x72, 0xfb,
      0x55, 0x99, 0x9f, 0x72, 0x71, 0xef, 0x00, 0x41,
      0x54, 0x0f, 0x2e, 0x60, 0xbe, 0x77, 0x71, 0x02,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__GetCoils_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x19, 0xab, 0x18, 0x88, 0xe7, 0x91, 0x54, 0xcb,
      0xca, 0xa4, 0x00, 0x0c, 0xc9, 0x35, 0xb7, 0x6b,
      0x69, 0xd3, 0x05, 0xfc, 0xb9, 0x78, 0xa3, 0x54,
      0x66, 0x24, 0xfb, 0xc6, 0x36, 0x09, 0xb7, 0xa8,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__GetCoils_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x64, 0xdb, 0x9f, 0x83, 0xef, 0x51, 0x14, 0xd0,
      0x65, 0x25, 0xd3, 0xaa, 0x8d, 0x2e, 0xac, 0xcc,
      0x6a, 0xe2, 0x8c, 0xb7, 0x71, 0xae, 0x15, 0x71,
      0x90, 0xcc, 0x8f, 0xc3, 0x96, 0xc5, 0x6d, 0x80,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__GetCoils_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xef, 0x09, 0x0b, 0x25, 0x34, 0x12, 0x85, 0xb6,
      0xcc, 0x16, 0x6f, 0xbb, 0xa8, 0x9a, 0x9b, 0x7a,
      0x02, 0x9d, 0x83, 0xb6, 0xb9, 0xbd, 0xd1, 0x0a,
      0x59, 0x34, 0x90, 0x63, 0x67, 0x5a, 0x71, 0xa5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char dobot_msgs_v4__srv__GetCoils__TYPE_NAME[] = "dobot_msgs_v4/srv/GetCoils";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char dobot_msgs_v4__srv__GetCoils_Event__TYPE_NAME[] = "dobot_msgs_v4/srv/GetCoils_Event";
static char dobot_msgs_v4__srv__GetCoils_Request__TYPE_NAME[] = "dobot_msgs_v4/srv/GetCoils_Request";
static char dobot_msgs_v4__srv__GetCoils_Response__TYPE_NAME[] = "dobot_msgs_v4/srv/GetCoils_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char dobot_msgs_v4__srv__GetCoils__FIELD_NAME__request_message[] = "request_message";
static char dobot_msgs_v4__srv__GetCoils__FIELD_NAME__response_message[] = "response_message";
static char dobot_msgs_v4__srv__GetCoils__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__GetCoils__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__GetCoils__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__GetCoils_Request__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__GetCoils_Response__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__GetCoils_Event__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dobot_msgs_v4__srv__GetCoils__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Event__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Request__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Response__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__GetCoils__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__GetCoils__TYPE_NAME, 26, 26},
      {dobot_msgs_v4__srv__GetCoils__FIELDS, 3, 3},
    },
    {dobot_msgs_v4__srv__GetCoils__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = dobot_msgs_v4__srv__GetCoils_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = dobot_msgs_v4__srv__GetCoils_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = dobot_msgs_v4__srv__GetCoils_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__GetCoils_Request__FIELD_NAME__index[] = "index";
static char dobot_msgs_v4__srv__GetCoils_Request__FIELD_NAME__addr[] = "addr";
static char dobot_msgs_v4__srv__GetCoils_Request__FIELD_NAME__count[] = "count";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__GetCoils_Request__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__GetCoils_Request__FIELD_NAME__index, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Request__FIELD_NAME__addr, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Request__FIELD_NAME__count, 5, 5},
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
dobot_msgs_v4__srv__GetCoils_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__GetCoils_Request__TYPE_NAME, 34, 34},
      {dobot_msgs_v4__srv__GetCoils_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__GetCoils_Response__FIELD_NAME__robot_return[] = "robot_return";
static char dobot_msgs_v4__srv__GetCoils_Response__FIELD_NAME__res[] = "res";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__GetCoils_Response__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__GetCoils_Response__FIELD_NAME__robot_return, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Response__FIELD_NAME__res, 3, 3},
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
dobot_msgs_v4__srv__GetCoils_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__GetCoils_Response__TYPE_NAME, 35, 35},
      {dobot_msgs_v4__srv__GetCoils_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__GetCoils_Event__FIELD_NAME__info[] = "info";
static char dobot_msgs_v4__srv__GetCoils_Event__FIELD_NAME__request[] = "request";
static char dobot_msgs_v4__srv__GetCoils_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__GetCoils_Event__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__GetCoils_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dobot_msgs_v4__srv__GetCoils_Request__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dobot_msgs_v4__srv__GetCoils_Response__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dobot_msgs_v4__srv__GetCoils_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Request__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__GetCoils_Response__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__GetCoils_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__GetCoils_Event__TYPE_NAME, 32, 32},
      {dobot_msgs_v4__srv__GetCoils_Event__FIELDS, 3, 3},
    },
    {dobot_msgs_v4__srv__GetCoils_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = dobot_msgs_v4__srv__GetCoils_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = dobot_msgs_v4__srv__GetCoils_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 index\n"
  "int32 addr\n"
  "int32 count\n"
  "---\n"
  "string robot_return\n"
  "int32 res";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__GetCoils__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__GetCoils__TYPE_NAME, 26, 26},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 68, 68},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__GetCoils_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__GetCoils_Request__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__GetCoils_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__GetCoils_Response__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__GetCoils_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__GetCoils_Event__TYPE_NAME, 32, 32},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__GetCoils__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__GetCoils__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *dobot_msgs_v4__srv__GetCoils_Event__get_individual_type_description_source(NULL);
    sources[3] = *dobot_msgs_v4__srv__GetCoils_Request__get_individual_type_description_source(NULL);
    sources[4] = *dobot_msgs_v4__srv__GetCoils_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__GetCoils_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__GetCoils_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__GetCoils_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__GetCoils_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__GetCoils_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__GetCoils_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *dobot_msgs_v4__srv__GetCoils_Request__get_individual_type_description_source(NULL);
    sources[3] = *dobot_msgs_v4__srv__GetCoils_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
