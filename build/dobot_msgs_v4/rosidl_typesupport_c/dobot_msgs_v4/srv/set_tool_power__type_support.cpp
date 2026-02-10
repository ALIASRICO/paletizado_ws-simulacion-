// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v4:srv/SetToolPower.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "dobot_msgs_v4/srv/detail/set_tool_power__struct.h"
#include "dobot_msgs_v4/srv/detail/set_tool_power__type_support.h"
#include "dobot_msgs_v4/srv/detail/set_tool_power__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _SetToolPower_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetToolPower_Request_type_support_ids_t;

static const _SetToolPower_Request_type_support_ids_t _SetToolPower_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetToolPower_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetToolPower_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetToolPower_Request_type_support_symbol_names_t _SetToolPower_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, SetToolPower_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, SetToolPower_Request)),
  }
};

typedef struct _SetToolPower_Request_type_support_data_t
{
  void * data[2];
} _SetToolPower_Request_type_support_data_t;

static _SetToolPower_Request_type_support_data_t _SetToolPower_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetToolPower_Request_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_SetToolPower_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SetToolPower_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SetToolPower_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetToolPower_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetToolPower_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__SetToolPower_Request__get_type_hash,
  &dobot_msgs_v4__srv__SetToolPower_Request__get_type_description,
  &dobot_msgs_v4__srv__SetToolPower_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, SetToolPower_Request)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::SetToolPower_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/set_tool_power__struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/set_tool_power__type_support.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/set_tool_power__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _SetToolPower_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetToolPower_Response_type_support_ids_t;

static const _SetToolPower_Response_type_support_ids_t _SetToolPower_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetToolPower_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetToolPower_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetToolPower_Response_type_support_symbol_names_t _SetToolPower_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, SetToolPower_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, SetToolPower_Response)),
  }
};

typedef struct _SetToolPower_Response_type_support_data_t
{
  void * data[2];
} _SetToolPower_Response_type_support_data_t;

static _SetToolPower_Response_type_support_data_t _SetToolPower_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetToolPower_Response_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_SetToolPower_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SetToolPower_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SetToolPower_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetToolPower_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetToolPower_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__SetToolPower_Response__get_type_hash,
  &dobot_msgs_v4__srv__SetToolPower_Response__get_type_description,
  &dobot_msgs_v4__srv__SetToolPower_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, SetToolPower_Response)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::SetToolPower_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/set_tool_power__struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/set_tool_power__type_support.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/set_tool_power__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _SetToolPower_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetToolPower_Event_type_support_ids_t;

static const _SetToolPower_Event_type_support_ids_t _SetToolPower_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetToolPower_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetToolPower_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetToolPower_Event_type_support_symbol_names_t _SetToolPower_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, SetToolPower_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, SetToolPower_Event)),
  }
};

typedef struct _SetToolPower_Event_type_support_data_t
{
  void * data[2];
} _SetToolPower_Event_type_support_data_t;

static _SetToolPower_Event_type_support_data_t _SetToolPower_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetToolPower_Event_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_SetToolPower_Event_message_typesupport_ids.typesupport_identifier[0],
  &_SetToolPower_Event_message_typesupport_symbol_names.symbol_name[0],
  &_SetToolPower_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetToolPower_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetToolPower_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__SetToolPower_Event__get_type_hash,
  &dobot_msgs_v4__srv__SetToolPower_Event__get_type_description,
  &dobot_msgs_v4__srv__SetToolPower_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, SetToolPower_Event)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::SetToolPower_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/set_tool_power__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _SetToolPower_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetToolPower_type_support_ids_t;

static const _SetToolPower_type_support_ids_t _SetToolPower_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SetToolPower_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetToolPower_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetToolPower_type_support_symbol_names_t _SetToolPower_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, SetToolPower)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, SetToolPower)),
  }
};

typedef struct _SetToolPower_type_support_data_t
{
  void * data[2];
} _SetToolPower_type_support_data_t;

static _SetToolPower_type_support_data_t _SetToolPower_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetToolPower_service_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_SetToolPower_service_typesupport_ids.typesupport_identifier[0],
  &_SetToolPower_service_typesupport_symbol_names.symbol_name[0],
  &_SetToolPower_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SetToolPower_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetToolPower_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &SetToolPower_Request_message_type_support_handle,
  &SetToolPower_Response_message_type_support_handle,
  &SetToolPower_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dobot_msgs_v4,
    srv,
    SetToolPower
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dobot_msgs_v4,
    srv,
    SetToolPower
  ),
  &dobot_msgs_v4__srv__SetToolPower__get_type_hash,
  &dobot_msgs_v4__srv__SetToolPower__get_type_description,
  &dobot_msgs_v4__srv__SetToolPower__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, SetToolPower)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::SetToolPower_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
