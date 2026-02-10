// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dobot_msgs_v4:srv/SetOutputBool.idl
// generated code does not contain a copyright notice

#include "dobot_msgs_v4/srv/detail/set_output_bool__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__SetOutputBool__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x81, 0x69, 0xd0, 0x50, 0xcc, 0x43, 0x4d, 0x84,
      0x8f, 0xe6, 0xbf, 0x3a, 0x63, 0xd3, 0x29, 0xf1,
      0x47, 0x68, 0x72, 0xc4, 0x92, 0xa7, 0x99, 0xdc,
      0x11, 0x68, 0x93, 0xec, 0x33, 0xec, 0x0a, 0x27,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__SetOutputBool_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x28, 0xd7, 0xca, 0x91, 0x71, 0x18, 0x02, 0x20,
      0x3e, 0x3b, 0xb8, 0xdb, 0x7b, 0xac, 0xca, 0x24,
      0x27, 0x71, 0xa6, 0x37, 0xcc, 0x1e, 0x95, 0x00,
      0xf1, 0x60, 0x13, 0x0a, 0x7a, 0x42, 0x21, 0x3f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__SetOutputBool_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe2, 0xf1, 0x85, 0x17, 0x20, 0xfa, 0x7d, 0x65,
      0x4c, 0x2e, 0x5d, 0x79, 0x7b, 0x55, 0x7b, 0x35,
      0xbb, 0xf1, 0xc3, 0xf1, 0x10, 0x4d, 0xf9, 0xab,
      0xa0, 0xcd, 0x62, 0x50, 0x28, 0xa1, 0x76, 0x98,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__SetOutputBool_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7d, 0xbc, 0x44, 0x95, 0xab, 0x2b, 0xd1, 0x9b,
      0x36, 0x76, 0x8b, 0x77, 0xc9, 0xf4, 0x60, 0x0c,
      0x59, 0x8b, 0xcf, 0xad, 0x45, 0x73, 0x49, 0xaa,
      0x18, 0xa1, 0xb1, 0x50, 0x05, 0x2d, 0xc8, 0xcf,
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

static char dobot_msgs_v4__srv__SetOutputBool__TYPE_NAME[] = "dobot_msgs_v4/srv/SetOutputBool";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char dobot_msgs_v4__srv__SetOutputBool_Event__TYPE_NAME[] = "dobot_msgs_v4/srv/SetOutputBool_Event";
static char dobot_msgs_v4__srv__SetOutputBool_Request__TYPE_NAME[] = "dobot_msgs_v4/srv/SetOutputBool_Request";
static char dobot_msgs_v4__srv__SetOutputBool_Response__TYPE_NAME[] = "dobot_msgs_v4/srv/SetOutputBool_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char dobot_msgs_v4__srv__SetOutputBool__FIELD_NAME__request_message[] = "request_message";
static char dobot_msgs_v4__srv__SetOutputBool__FIELD_NAME__response_message[] = "response_message";
static char dobot_msgs_v4__srv__SetOutputBool__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__SetOutputBool__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__SetOutputBool__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__SetOutputBool_Request__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__SetOutputBool_Response__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dobot_msgs_v4__srv__SetOutputBool_Event__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dobot_msgs_v4__srv__SetOutputBool__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool_Event__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool_Request__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool_Response__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__SetOutputBool__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__SetOutputBool__TYPE_NAME, 31, 31},
      {dobot_msgs_v4__srv__SetOutputBool__FIELDS, 3, 3},
    },
    {dobot_msgs_v4__srv__SetOutputBool__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = dobot_msgs_v4__srv__SetOutputBool_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = dobot_msgs_v4__srv__SetOutputBool_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = dobot_msgs_v4__srv__SetOutputBool_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__SetOutputBool_Request__FIELD_NAME__address[] = "address";
static char dobot_msgs_v4__srv__SetOutputBool_Request__FIELD_NAME__value[] = "value";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__SetOutputBool_Request__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__SetOutputBool_Request__FIELD_NAME__address, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool_Request__FIELD_NAME__value, 5, 5},
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
dobot_msgs_v4__srv__SetOutputBool_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__SetOutputBool_Request__TYPE_NAME, 39, 39},
      {dobot_msgs_v4__srv__SetOutputBool_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__SetOutputBool_Response__FIELD_NAME__res[] = "res";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__SetOutputBool_Response__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__SetOutputBool_Response__FIELD_NAME__res, 3, 3},
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
dobot_msgs_v4__srv__SetOutputBool_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__SetOutputBool_Response__TYPE_NAME, 40, 40},
      {dobot_msgs_v4__srv__SetOutputBool_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dobot_msgs_v4__srv__SetOutputBool_Event__FIELD_NAME__info[] = "info";
static char dobot_msgs_v4__srv__SetOutputBool_Event__FIELD_NAME__request[] = "request";
static char dobot_msgs_v4__srv__SetOutputBool_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field dobot_msgs_v4__srv__SetOutputBool_Event__FIELDS[] = {
  {
    {dobot_msgs_v4__srv__SetOutputBool_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dobot_msgs_v4__srv__SetOutputBool_Request__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dobot_msgs_v4__srv__SetOutputBool_Response__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dobot_msgs_v4__srv__SetOutputBool_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool_Request__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {dobot_msgs_v4__srv__SetOutputBool_Response__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__SetOutputBool_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dobot_msgs_v4__srv__SetOutputBool_Event__TYPE_NAME, 37, 37},
      {dobot_msgs_v4__srv__SetOutputBool_Event__FIELDS, 3, 3},
    },
    {dobot_msgs_v4__srv__SetOutputBool_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = dobot_msgs_v4__srv__SetOutputBool_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = dobot_msgs_v4__srv__SetOutputBool_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 address\n"
  "int32 value\n"
  "---\n"
  "int32 res";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__SetOutputBool__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__SetOutputBool__TYPE_NAME, 31, 31},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 39, 39},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__SetOutputBool_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__SetOutputBool_Request__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__SetOutputBool_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__SetOutputBool_Response__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__SetOutputBool_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dobot_msgs_v4__srv__SetOutputBool_Event__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__SetOutputBool__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__SetOutputBool__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *dobot_msgs_v4__srv__SetOutputBool_Event__get_individual_type_description_source(NULL);
    sources[3] = *dobot_msgs_v4__srv__SetOutputBool_Request__get_individual_type_description_source(NULL);
    sources[4] = *dobot_msgs_v4__srv__SetOutputBool_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__SetOutputBool_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__SetOutputBool_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__SetOutputBool_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__SetOutputBool_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__SetOutputBool_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dobot_msgs_v4__srv__SetOutputBool_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *dobot_msgs_v4__srv__SetOutputBool_Request__get_individual_type_description_source(NULL);
    sources[3] = *dobot_msgs_v4__srv__SetOutputBool_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
