// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dobot_msgs_v4:srv/Tool.idl
// generated code does not contain a copyright notice

#include "dobot_msgs_v4/srv/detail/tool__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__Tool__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5d, 0x7e, 0xe0, 0xf6, 0xb8, 0x6f, 0x48, 0x0f,
      0x94, 0xb3, 0x9e, 0x6d, 0x33, 0xf9, 0x97, 0x79,
      0x6f, 0xcf, 0x56, 0x4e, 0x02, 0x32, 0xe4, 0xdb,
      0x95, 0x54, 0x14, 0x78, 0x2f, 0x1f, 0xaa, 0x1a,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__Tool_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9e, 0x77, 0x72, 0x83, 0x2d, 0xe9, 0x26, 0x02,
      0x68, 0xe9, 0x19, 0xd7, 0x2e, 0xa2, 0xfb, 0xe5,
      0xdb, 0xd6, 0x40, 0x81, 0x86, 0x58, 0x30, 0xe2,
      0x6b, 0x62, 0x4c, 0x0b, 0xda, 0x6b, 0x31, 0x87,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__Tool_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x08, 0xf0, 0xfa, 0xab, 0xcf, 0xe1, 0x12, 0xca,
      0xa8, 0xfc, 0x70, 0xbf, 0xa9, 0xc8, 0xd1, 0xca,
      0xbb, 0x1e, 0xcb, 0x80, 0x7a, 0x21, 0x73, 0x4f,
      0x75, 0xd7, 0x7f, 0x91, 0x04, 0x61, 0x8d, 0x3b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__Tool_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x09, 0xc1, 0xb4, 0x15, 0x47, 0x62, 0x39, 0x05,
      0xb2, 0xd7, 0xc1, 0x72, 0x36, 0xe1, 0xd6, 0x85,
      0x00, 0x4a, 0x5f, 0x29, 0x6c, 0x13, 0x72, 0x1a,
      0x91, 0x2d, 0xe8, 0x00, 0xff, 0xc4, 0xd7, 0x7c,
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

static char dobot_msgs_v4__srv__Tool__TYPE_NAME[] = "dobot_msgs_v4/srv/Tool";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char dobot_msgs_v4__srv__Tool_Event__TYPE_NAME[] = "dobot_msgs_v4/srv/Tool_Event";
static char dobot_msgs_v4__srv__Tool_Request__TYPE_NAME[] = "dobot_msgs_v4/srv/Tool_Request";
static char dobot_msgs_v4__srv__Tool_Response__TYPE_NAME[] = "dobot_msgs_v4/srv/Tool_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char dobot_msgs_v4__srv__Tool__FIELD_NAME__request_message[] = "request_message";
static char dobot_msgs_v4__srv__Tool__FIELD_NAME__response_message[] = "response_message";
static char dobot_msgs_v4__srv__Tool__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__Tool__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__Tool__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__Tool_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__Tool__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__Tool_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__Tool__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__Tool_Event__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dobot_msgs_v4__srv__Tool__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__Tool_Event__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__Tool_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__Tool_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__Tool__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__Tool__TYPE_NAME, 22, 22},
      {dobot_msgs_v4__srv__Tool__FIELDS, 3, 3},
    },
    {dobot_msgs_v4__srv__Tool__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = dobot_msgs_v4__srv__Tool_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = dobot_msgs_v4__srv__Tool_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = dobot_msgs_v4__srv__Tool_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__Tool_Request__FIELD_NAME__index[] = "index";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__Tool_Request__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__Tool_Request__FIELD_NAME__index, 5, 5},
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
dobot_msgs_v4__srv__Tool_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__Tool_Request__TYPE_NAME, 30, 30},
      {dobot_msgs_v4__srv__Tool_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__Tool_Response__FIELD_NAME__res[] = "res";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__Tool_Response__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__Tool_Response__FIELD_NAME__res, 3, 3},
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
dobot_msgs_v4__srv__Tool_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__Tool_Response__TYPE_NAME, 31, 31},
      {dobot_msgs_v4__srv__Tool_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__Tool_Event__FIELD_NAME__info[] = "info";
static char dobot_msgs_v4__srv__Tool_Event__FIELD_NAME__request[] = "request";
static char dobot_msgs_v4__srv__Tool_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__Tool_Event__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__Tool_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__Tool_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dobot_msgs_v4__srv__Tool_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__Tool_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dobot_msgs_v4__srv__Tool_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dobot_msgs_v4__srv__Tool_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__Tool_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__Tool_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__Tool_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__Tool_Event__TYPE_NAME, 28, 28},
      {dobot_msgs_v4__srv__Tool_Event__FIELDS, 3, 3},
    },
    {dobot_msgs_v4__srv__Tool_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = dobot_msgs_v4__srv__Tool_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = dobot_msgs_v4__srv__Tool_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 index\n"
  "---\n"
  "int32 res";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__Tool__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__Tool__TYPE_NAME, 22, 22},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 25, 25},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__Tool_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__Tool_Request__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__Tool_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__Tool_Response__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__Tool_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__Tool_Event__TYPE_NAME, 28, 28},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__Tool__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__Tool__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *dobot_msgs_v4__srv__Tool_Event__get_individual_type_description_source(NULL);
    sources[3] = *dobot_msgs_v4__srv__Tool_Request__get_individual_type_description_source(NULL);
    sources[4] = *dobot_msgs_v4__srv__Tool_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__Tool_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__Tool_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__Tool_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__Tool_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__Tool_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__Tool_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *dobot_msgs_v4__srv__Tool_Request__get_individual_type_description_source(NULL);
    sources[3] = *dobot_msgs_v4__srv__Tool_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
