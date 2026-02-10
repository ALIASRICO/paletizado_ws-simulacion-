// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dobot_msgs_v4:srv/FCForceMode.idl
// generated code does not contain a copyright notice

#include "dobot_msgs_v4/srv/detail/fc_force_mode__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__FCForceMode__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbe, 0x58, 0x86, 0xac, 0x52, 0x75, 0x39, 0xac,
      0x2d, 0x03, 0x03, 0xf9, 0x13, 0x7d, 0xeb, 0x77,
      0x41, 0xdd, 0xeb, 0x88, 0x1d, 0x3d, 0x25, 0xb6,
      0xa1, 0xc7, 0x81, 0xa1, 0xac, 0x1a, 0xbb, 0xd5,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__FCForceMode_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7b, 0x35, 0x7f, 0x3a, 0xfa, 0x5c, 0x35, 0x1a,
      0x56, 0xe2, 0x0c, 0x26, 0xe6, 0x4d, 0xa0, 0x61,
      0xde, 0xd0, 0xd8, 0xa7, 0xcb, 0x22, 0xa0, 0x5d,
      0xe1, 0xae, 0x16, 0x1b, 0xc7, 0xfe, 0x37, 0x22,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__FCForceMode_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcd, 0x07, 0xe9, 0x77, 0xeb, 0xaf, 0x9c, 0x15,
      0x66, 0xdd, 0x58, 0x7d, 0x87, 0x28, 0xaf, 0x6b,
      0x97, 0xed, 0x46, 0x54, 0x36, 0x78, 0xbf, 0x5c,
      0x76, 0x0c, 0xa3, 0x2f, 0x77, 0xb8, 0x5c, 0xc2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__FCForceMode_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xaf, 0x76, 0xa3, 0x5a, 0x03, 0x0f, 0x46, 0xaf,
      0x07, 0x84, 0xc4, 0x4a, 0x80, 0x88, 0x6f, 0xf0,
      0x8d, 0x36, 0x27, 0x96, 0x1e, 0xcc, 0xa5, 0xb6,
      0x46, 0x88, 0x2f, 0x22, 0x3a, 0xcf, 0x50, 0x50,
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

static char dobot_msgs_v4__srv__FCForceMode__TYPE_NAME[] = "dobot_msgs_v4/srv/FCForceMode";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char dobot_msgs_v4__srv__FCForceMode_Event__TYPE_NAME[] = "dobot_msgs_v4/srv/FCForceMode_Event";
static char dobot_msgs_v4__srv__FCForceMode_Request__TYPE_NAME[] = "dobot_msgs_v4/srv/FCForceMode_Request";
static char dobot_msgs_v4__srv__FCForceMode_Response__TYPE_NAME[] = "dobot_msgs_v4/srv/FCForceMode_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char dobot_msgs_v4__srv__FCForceMode__FIELD_NAME__request_message[] = "request_message";
static char dobot_msgs_v4__srv__FCForceMode__FIELD_NAME__response_message[] = "response_message";
static char dobot_msgs_v4__srv__FCForceMode__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__FCForceMode__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__FCForceMode__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__FCForceMode_Request__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__FCForceMode_Response__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__FCForceMode_Event__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dobot_msgs_v4__srv__FCForceMode__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Event__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Response__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__FCForceMode__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__FCForceMode__TYPE_NAME, 29, 29},
      {dobot_msgs_v4__srv__FCForceMode__FIELDS, 3, 3},
    },
    {dobot_msgs_v4__srv__FCForceMode__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = dobot_msgs_v4__srv__FCForceMode_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = dobot_msgs_v4__srv__FCForceMode_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = dobot_msgs_v4__srv__FCForceMode_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__x[] = "x";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__y[] = "y";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__z[] = "z";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__rx[] = "rx";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__ry[] = "ry";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__rz[] = "rz";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__fx[] = "fx";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__fy[] = "fy";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__fz[] = "fz";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__frx[] = "frx";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__fry[] = "fry";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__frz[] = "frz";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__reference[] = "reference";
static char dobot_msgs_v4__srv__FCForceMode_Request__DEFAULT_VALUE__reference[] = "-1";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__user[] = "user";
static char dobot_msgs_v4__srv__FCForceMode_Request__DEFAULT_VALUE__user[] = "-1";
static char dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__tool[] = "tool";
static char dobot_msgs_v4__srv__FCForceMode_Request__DEFAULT_VALUE__tool[] = "-1";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__FCForceMode_Request__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__rx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__ry, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__rz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__fx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__fy, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__fz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__frx, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__fry, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__frz, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__reference, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {dobot_msgs_v4__srv__FCForceMode_Request__DEFAULT_VALUE__reference, 2, 2},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__user, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {dobot_msgs_v4__srv__FCForceMode_Request__DEFAULT_VALUE__user, 2, 2},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__FIELD_NAME__tool, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {dobot_msgs_v4__srv__FCForceMode_Request__DEFAULT_VALUE__tool, 2, 2},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__FCForceMode_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__FCForceMode_Request__TYPE_NAME, 37, 37},
      {dobot_msgs_v4__srv__FCForceMode_Request__FIELDS, 15, 15},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__FCForceMode_Response__FIELD_NAME__res[] = "res";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__FCForceMode_Response__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__FCForceMode_Response__FIELD_NAME__res, 3, 3},
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
dobot_msgs_v4__srv__FCForceMode_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__FCForceMode_Response__TYPE_NAME, 38, 38},
      {dobot_msgs_v4__srv__FCForceMode_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__FCForceMode_Event__FIELD_NAME__info[] = "info";
static char dobot_msgs_v4__srv__FCForceMode_Event__FIELD_NAME__request[] = "request";
static char dobot_msgs_v4__srv__FCForceMode_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__FCForceMode_Event__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__FCForceMode_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dobot_msgs_v4__srv__FCForceMode_Request__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dobot_msgs_v4__srv__FCForceMode_Response__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dobot_msgs_v4__srv__FCForceMode_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Request__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__FCForceMode_Response__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__FCForceMode_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__FCForceMode_Event__TYPE_NAME, 35, 35},
      {dobot_msgs_v4__srv__FCForceMode_Event__FIELDS, 3, 3},
    },
    {dobot_msgs_v4__srv__FCForceMode_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = dobot_msgs_v4__srv__FCForceMode_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = dobot_msgs_v4__srv__FCForceMode_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 x\n"
  "int32 y\n"
  "int32 z\n"
  "int32 rx\n"
  "int32 ry\n"
  "int32 rz\n"
  "int32 fx\n"
  "int32 fy\n"
  "int32 fz\n"
  "int32 frx\n"
  "int32 fry\n"
  "int32 frz\n"
  "int32 reference -1\n"
  "int32 user -1\n"
  "int32 tool -1\n"
  "---\n"
  "int32 res";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__FCForceMode__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__FCForceMode__TYPE_NAME, 29, 29},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 169, 169},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__FCForceMode_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__FCForceMode_Request__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__FCForceMode_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__FCForceMode_Response__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__FCForceMode_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__FCForceMode_Event__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__FCForceMode__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__FCForceMode__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *dobot_msgs_v4__srv__FCForceMode_Event__get_individual_type_description_source(NULL);
    sources[3] = *dobot_msgs_v4__srv__FCForceMode_Request__get_individual_type_description_source(NULL);
    sources[4] = *dobot_msgs_v4__srv__FCForceMode_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__FCForceMode_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__FCForceMode_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__FCForceMode_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__FCForceMode_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__FCForceMode_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__FCForceMode_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *dobot_msgs_v4__srv__FCForceMode_Request__get_individual_type_description_source(NULL);
    sources[3] = *dobot_msgs_v4__srv__FCForceMode_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
