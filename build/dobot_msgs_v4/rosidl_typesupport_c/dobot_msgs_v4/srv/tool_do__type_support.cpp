// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v4:srv/ToolDO.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "dobot_msgs_v4/srv/detail/tool_do__struct.h"
#include "dobot_msgs_v4/srv/detail/tool_do__type_support.h"
#include "dobot_msgs_v4/srv/detail/tool_do__functions.h"
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

typedef struct _ToolDO_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ToolDO_Request_type_support_ids_t;

static const _ToolDO_Request_type_support_ids_t _ToolDO_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ToolDO_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ToolDO_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ToolDO_Request_type_support_symbol_names_t _ToolDO_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, ToolDO_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, ToolDO_Request)),
  }
};

typedef struct _ToolDO_Request_type_support_data_t
{
  void * data[2];
} _ToolDO_Request_type_support_data_t;

static _ToolDO_Request_type_support_data_t _ToolDO_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ToolDO_Request_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_ToolDO_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ToolDO_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ToolDO_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ToolDO_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ToolDO_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__ToolDO_Request__get_type_hash,
  &dobot_msgs_v4__srv__ToolDO_Request__get_type_description,
  &dobot_msgs_v4__srv__ToolDO_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, ToolDO_Request)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::ToolDO_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/tool_do__struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/tool_do__type_support.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/tool_do__functions.h"
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

typedef struct _ToolDO_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ToolDO_Response_type_support_ids_t;

static const _ToolDO_Response_type_support_ids_t _ToolDO_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ToolDO_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ToolDO_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ToolDO_Response_type_support_symbol_names_t _ToolDO_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, ToolDO_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, ToolDO_Response)),
  }
};

typedef struct _ToolDO_Response_type_support_data_t
{
  void * data[2];
} _ToolDO_Response_type_support_data_t;

static _ToolDO_Response_type_support_data_t _ToolDO_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ToolDO_Response_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_ToolDO_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ToolDO_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ToolDO_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ToolDO_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ToolDO_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__ToolDO_Response__get_type_hash,
  &dobot_msgs_v4__srv__ToolDO_Response__get_type_description,
  &dobot_msgs_v4__srv__ToolDO_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, ToolDO_Response)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::ToolDO_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/tool_do__struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/tool_do__type_support.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/tool_do__functions.h"
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

typedef struct _ToolDO_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ToolDO_Event_type_support_ids_t;

static const _ToolDO_Event_type_support_ids_t _ToolDO_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ToolDO_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ToolDO_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ToolDO_Event_type_support_symbol_names_t _ToolDO_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, ToolDO_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, ToolDO_Event)),
  }
};

typedef struct _ToolDO_Event_type_support_data_t
{
  void * data[2];
} _ToolDO_Event_type_support_data_t;

static _ToolDO_Event_type_support_data_t _ToolDO_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ToolDO_Event_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_ToolDO_Event_message_typesupport_ids.typesupport_identifier[0],
  &_ToolDO_Event_message_typesupport_symbol_names.symbol_name[0],
  &_ToolDO_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ToolDO_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ToolDO_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__ToolDO_Event__get_type_hash,
  &dobot_msgs_v4__srv__ToolDO_Event__get_type_description,
  &dobot_msgs_v4__srv__ToolDO_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, ToolDO_Event)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::ToolDO_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/tool_do__type_support.h"
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
typedef struct _ToolDO_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ToolDO_type_support_ids_t;

static const _ToolDO_type_support_ids_t _ToolDO_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ToolDO_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ToolDO_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ToolDO_type_support_symbol_names_t _ToolDO_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, ToolDO)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, ToolDO)),
  }
};

typedef struct _ToolDO_type_support_data_t
{
  void * data[2];
} _ToolDO_type_support_data_t;

static _ToolDO_type_support_data_t _ToolDO_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ToolDO_service_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_ToolDO_service_typesupport_ids.typesupport_identifier[0],
  &_ToolDO_service_typesupport_symbol_names.symbol_name[0],
  &_ToolDO_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ToolDO_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ToolDO_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &ToolDO_Request_message_type_support_handle,
  &ToolDO_Response_message_type_support_handle,
  &ToolDO_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dobot_msgs_v4,
    srv,
    ToolDO
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dobot_msgs_v4,
    srv,
    ToolDO
  ),
  &dobot_msgs_v4__srv__ToolDO__get_type_hash,
  &dobot_msgs_v4__srv__ToolDO__get_type_description,
  &dobot_msgs_v4__srv__ToolDO__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, ToolDO)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::ToolDO_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
