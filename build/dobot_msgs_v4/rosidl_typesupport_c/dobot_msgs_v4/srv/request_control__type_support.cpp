// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v4:srv/RequestControl.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "dobot_msgs_v4/srv/detail/request_control__struct.h"
#include "dobot_msgs_v4/srv/detail/request_control__type_support.h"
#include "dobot_msgs_v4/srv/detail/request_control__functions.h"
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

typedef struct _RequestControl_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RequestControl_Request_type_support_ids_t;

static const _RequestControl_Request_type_support_ids_t _RequestControl_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RequestControl_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RequestControl_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RequestControl_Request_type_support_symbol_names_t _RequestControl_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, RequestControl_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, RequestControl_Request)),
  }
};

typedef struct _RequestControl_Request_type_support_data_t
{
  void * data[2];
} _RequestControl_Request_type_support_data_t;

static _RequestControl_Request_type_support_data_t _RequestControl_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RequestControl_Request_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_RequestControl_Request_message_typesupport_ids.typesupport_identifier[0],
  &_RequestControl_Request_message_typesupport_symbol_names.symbol_name[0],
  &_RequestControl_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RequestControl_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RequestControl_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__RequestControl_Request__get_type_hash,
  &dobot_msgs_v4__srv__RequestControl_Request__get_type_description,
  &dobot_msgs_v4__srv__RequestControl_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, RequestControl_Request)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::RequestControl_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/request_control__struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/request_control__type_support.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/request_control__functions.h"
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

typedef struct _RequestControl_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RequestControl_Response_type_support_ids_t;

static const _RequestControl_Response_type_support_ids_t _RequestControl_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RequestControl_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RequestControl_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RequestControl_Response_type_support_symbol_names_t _RequestControl_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, RequestControl_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, RequestControl_Response)),
  }
};

typedef struct _RequestControl_Response_type_support_data_t
{
  void * data[2];
} _RequestControl_Response_type_support_data_t;

static _RequestControl_Response_type_support_data_t _RequestControl_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RequestControl_Response_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_RequestControl_Response_message_typesupport_ids.typesupport_identifier[0],
  &_RequestControl_Response_message_typesupport_symbol_names.symbol_name[0],
  &_RequestControl_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RequestControl_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RequestControl_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__RequestControl_Response__get_type_hash,
  &dobot_msgs_v4__srv__RequestControl_Response__get_type_description,
  &dobot_msgs_v4__srv__RequestControl_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, RequestControl_Response)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::RequestControl_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/request_control__struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/request_control__type_support.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/request_control__functions.h"
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

typedef struct _RequestControl_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RequestControl_Event_type_support_ids_t;

static const _RequestControl_Event_type_support_ids_t _RequestControl_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RequestControl_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RequestControl_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RequestControl_Event_type_support_symbol_names_t _RequestControl_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, RequestControl_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, RequestControl_Event)),
  }
};

typedef struct _RequestControl_Event_type_support_data_t
{
  void * data[2];
} _RequestControl_Event_type_support_data_t;

static _RequestControl_Event_type_support_data_t _RequestControl_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RequestControl_Event_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_RequestControl_Event_message_typesupport_ids.typesupport_identifier[0],
  &_RequestControl_Event_message_typesupport_symbol_names.symbol_name[0],
  &_RequestControl_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RequestControl_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RequestControl_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__RequestControl_Event__get_type_hash,
  &dobot_msgs_v4__srv__RequestControl_Event__get_type_description,
  &dobot_msgs_v4__srv__RequestControl_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, RequestControl_Event)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::RequestControl_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/request_control__type_support.h"
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
typedef struct _RequestControl_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RequestControl_type_support_ids_t;

static const _RequestControl_type_support_ids_t _RequestControl_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _RequestControl_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RequestControl_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RequestControl_type_support_symbol_names_t _RequestControl_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, srv, RequestControl)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, RequestControl)),
  }
};

typedef struct _RequestControl_type_support_data_t
{
  void * data[2];
} _RequestControl_type_support_data_t;

static _RequestControl_type_support_data_t _RequestControl_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RequestControl_service_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_RequestControl_service_typesupport_ids.typesupport_identifier[0],
  &_RequestControl_service_typesupport_symbol_names.symbol_name[0],
  &_RequestControl_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t RequestControl_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RequestControl_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &RequestControl_Request_message_type_support_handle,
  &RequestControl_Response_message_type_support_handle,
  &RequestControl_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dobot_msgs_v4,
    srv,
    RequestControl
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dobot_msgs_v4,
    srv,
    RequestControl
  ),
  &dobot_msgs_v4__srv__RequestControl__get_type_hash,
  &dobot_msgs_v4__srv__RequestControl__get_type_description,
  &dobot_msgs_v4__srv__RequestControl__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace dobot_msgs_v4

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, dobot_msgs_v4, srv, RequestControl)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_c::RequestControl_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
