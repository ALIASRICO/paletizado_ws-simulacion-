// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v4:srv/FCSetStiffness.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "dobot_msgs_v4/srv/detail/fc_set_stiffness__functions.h"
#include "dobot_msgs_v4/srv/detail/fc_set_stiffness__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _FCSetStiffness_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FCSetStiffness_Request_type_support_ids_t;

static const _FCSetStiffness_Request_type_support_ids_t _FCSetStiffness_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FCSetStiffness_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FCSetStiffness_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FCSetStiffness_Request_type_support_symbol_names_t _FCSetStiffness_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dobot_msgs_v4, srv, FCSetStiffness_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCSetStiffness_Request)),
  }
};

typedef struct _FCSetStiffness_Request_type_support_data_t
{
  void * data[2];
} _FCSetStiffness_Request_type_support_data_t;

static _FCSetStiffness_Request_type_support_data_t _FCSetStiffness_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FCSetStiffness_Request_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_FCSetStiffness_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FCSetStiffness_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FCSetStiffness_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FCSetStiffness_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FCSetStiffness_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__FCSetStiffness_Request__get_type_hash,
  &dobot_msgs_v4__srv__FCSetStiffness_Request__get_type_description,
  &dobot_msgs_v4__srv__FCSetStiffness_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness_Request>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_cpp::FCSetStiffness_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, dobot_msgs_v4, srv, FCSetStiffness_Request)() {
  return get_message_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_set_stiffness__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_set_stiffness__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _FCSetStiffness_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FCSetStiffness_Response_type_support_ids_t;

static const _FCSetStiffness_Response_type_support_ids_t _FCSetStiffness_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FCSetStiffness_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FCSetStiffness_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FCSetStiffness_Response_type_support_symbol_names_t _FCSetStiffness_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dobot_msgs_v4, srv, FCSetStiffness_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCSetStiffness_Response)),
  }
};

typedef struct _FCSetStiffness_Response_type_support_data_t
{
  void * data[2];
} _FCSetStiffness_Response_type_support_data_t;

static _FCSetStiffness_Response_type_support_data_t _FCSetStiffness_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FCSetStiffness_Response_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_FCSetStiffness_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FCSetStiffness_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FCSetStiffness_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FCSetStiffness_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FCSetStiffness_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__FCSetStiffness_Response__get_type_hash,
  &dobot_msgs_v4__srv__FCSetStiffness_Response__get_type_description,
  &dobot_msgs_v4__srv__FCSetStiffness_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness_Response>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_cpp::FCSetStiffness_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, dobot_msgs_v4, srv, FCSetStiffness_Response)() {
  return get_message_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_set_stiffness__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_set_stiffness__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _FCSetStiffness_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FCSetStiffness_Event_type_support_ids_t;

static const _FCSetStiffness_Event_type_support_ids_t _FCSetStiffness_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FCSetStiffness_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FCSetStiffness_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FCSetStiffness_Event_type_support_symbol_names_t _FCSetStiffness_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dobot_msgs_v4, srv, FCSetStiffness_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCSetStiffness_Event)),
  }
};

typedef struct _FCSetStiffness_Event_type_support_data_t
{
  void * data[2];
} _FCSetStiffness_Event_type_support_data_t;

static _FCSetStiffness_Event_type_support_data_t _FCSetStiffness_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FCSetStiffness_Event_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_FCSetStiffness_Event_message_typesupport_ids.typesupport_identifier[0],
  &_FCSetStiffness_Event_message_typesupport_symbol_names.symbol_name[0],
  &_FCSetStiffness_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FCSetStiffness_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FCSetStiffness_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__FCSetStiffness_Event__get_type_hash,
  &dobot_msgs_v4__srv__FCSetStiffness_Event__get_type_description,
  &dobot_msgs_v4__srv__FCSetStiffness_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness_Event>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_cpp::FCSetStiffness_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, dobot_msgs_v4, srv, FCSetStiffness_Event)() {
  return get_message_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_set_stiffness__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _FCSetStiffness_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FCSetStiffness_type_support_ids_t;

static const _FCSetStiffness_type_support_ids_t _FCSetStiffness_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FCSetStiffness_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FCSetStiffness_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FCSetStiffness_type_support_symbol_names_t _FCSetStiffness_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dobot_msgs_v4, srv, FCSetStiffness)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCSetStiffness)),
  }
};

typedef struct _FCSetStiffness_type_support_data_t
{
  void * data[2];
} _FCSetStiffness_type_support_data_t;

static _FCSetStiffness_type_support_data_t _FCSetStiffness_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FCSetStiffness_service_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_FCSetStiffness_service_typesupport_ids.typesupport_identifier[0],
  &_FCSetStiffness_service_typesupport_symbol_names.symbol_name[0],
  &_FCSetStiffness_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FCSetStiffness_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FCSetStiffness_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<dobot_msgs_v4::srv::FCSetStiffness>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<dobot_msgs_v4::srv::FCSetStiffness>,
  &dobot_msgs_v4__srv__FCSetStiffness__get_type_hash,
  &dobot_msgs_v4__srv__FCSetStiffness__get_type_description,
  &dobot_msgs_v4__srv__FCSetStiffness__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_cpp::FCSetStiffness_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, dobot_msgs_v4, srv, FCSetStiffness)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<dobot_msgs_v4::srv::FCSetStiffness>();
}

#ifdef __cplusplus
}
#endif
