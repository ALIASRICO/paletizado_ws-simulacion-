// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v4:srv/FCSetDamping.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "dobot_msgs_v4/srv/detail/fc_set_damping__functions.h"
#include "dobot_msgs_v4/srv/detail/fc_set_damping__struct.hpp"
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

typedef struct _FCSetDamping_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FCSetDamping_Request_type_support_ids_t;

static const _FCSetDamping_Request_type_support_ids_t _FCSetDamping_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FCSetDamping_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FCSetDamping_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FCSetDamping_Request_type_support_symbol_names_t _FCSetDamping_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dobot_msgs_v4, srv, FCSetDamping_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCSetDamping_Request)),
  }
};

typedef struct _FCSetDamping_Request_type_support_data_t
{
  void * data[2];
} _FCSetDamping_Request_type_support_data_t;

static _FCSetDamping_Request_type_support_data_t _FCSetDamping_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FCSetDamping_Request_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_FCSetDamping_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FCSetDamping_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FCSetDamping_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FCSetDamping_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FCSetDamping_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__FCSetDamping_Request__get_type_hash,
  &dobot_msgs_v4__srv__FCSetDamping_Request__get_type_description,
  &dobot_msgs_v4__srv__FCSetDamping_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::FCSetDamping_Request>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_cpp::FCSetDamping_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, dobot_msgs_v4, srv, FCSetDamping_Request)() {
  return get_message_type_support_handle<dobot_msgs_v4::srv::FCSetDamping_Request>();
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
// #include "dobot_msgs_v4/srv/detail/fc_set_damping__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_set_damping__struct.hpp"
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

typedef struct _FCSetDamping_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FCSetDamping_Response_type_support_ids_t;

static const _FCSetDamping_Response_type_support_ids_t _FCSetDamping_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FCSetDamping_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FCSetDamping_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FCSetDamping_Response_type_support_symbol_names_t _FCSetDamping_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dobot_msgs_v4, srv, FCSetDamping_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCSetDamping_Response)),
  }
};

typedef struct _FCSetDamping_Response_type_support_data_t
{
  void * data[2];
} _FCSetDamping_Response_type_support_data_t;

static _FCSetDamping_Response_type_support_data_t _FCSetDamping_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FCSetDamping_Response_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_FCSetDamping_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FCSetDamping_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FCSetDamping_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FCSetDamping_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FCSetDamping_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__FCSetDamping_Response__get_type_hash,
  &dobot_msgs_v4__srv__FCSetDamping_Response__get_type_description,
  &dobot_msgs_v4__srv__FCSetDamping_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::FCSetDamping_Response>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_cpp::FCSetDamping_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, dobot_msgs_v4, srv, FCSetDamping_Response)() {
  return get_message_type_support_handle<dobot_msgs_v4::srv::FCSetDamping_Response>();
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
// #include "dobot_msgs_v4/srv/detail/fc_set_damping__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_set_damping__struct.hpp"
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

typedef struct _FCSetDamping_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FCSetDamping_Event_type_support_ids_t;

static const _FCSetDamping_Event_type_support_ids_t _FCSetDamping_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FCSetDamping_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FCSetDamping_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FCSetDamping_Event_type_support_symbol_names_t _FCSetDamping_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dobot_msgs_v4, srv, FCSetDamping_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCSetDamping_Event)),
  }
};

typedef struct _FCSetDamping_Event_type_support_data_t
{
  void * data[2];
} _FCSetDamping_Event_type_support_data_t;

static _FCSetDamping_Event_type_support_data_t _FCSetDamping_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FCSetDamping_Event_message_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_FCSetDamping_Event_message_typesupport_ids.typesupport_identifier[0],
  &_FCSetDamping_Event_message_typesupport_symbol_names.symbol_name[0],
  &_FCSetDamping_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FCSetDamping_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FCSetDamping_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__FCSetDamping_Event__get_type_hash,
  &dobot_msgs_v4__srv__FCSetDamping_Event__get_type_description,
  &dobot_msgs_v4__srv__FCSetDamping_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::FCSetDamping_Event>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_cpp::FCSetDamping_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, dobot_msgs_v4, srv, FCSetDamping_Event)() {
  return get_message_type_support_handle<dobot_msgs_v4::srv::FCSetDamping_Event>();
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
// #include "dobot_msgs_v4/srv/detail/fc_set_damping__struct.hpp"
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

typedef struct _FCSetDamping_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FCSetDamping_type_support_ids_t;

static const _FCSetDamping_type_support_ids_t _FCSetDamping_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _FCSetDamping_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FCSetDamping_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FCSetDamping_type_support_symbol_names_t _FCSetDamping_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dobot_msgs_v4, srv, FCSetDamping)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCSetDamping)),
  }
};

typedef struct _FCSetDamping_type_support_data_t
{
  void * data[2];
} _FCSetDamping_type_support_data_t;

static _FCSetDamping_type_support_data_t _FCSetDamping_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FCSetDamping_service_typesupport_map = {
  2,
  "dobot_msgs_v4",
  &_FCSetDamping_service_typesupport_ids.typesupport_identifier[0],
  &_FCSetDamping_service_typesupport_symbol_names.symbol_name[0],
  &_FCSetDamping_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FCSetDamping_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FCSetDamping_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCSetDamping_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCSetDamping_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCSetDamping_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<dobot_msgs_v4::srv::FCSetDamping>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<dobot_msgs_v4::srv::FCSetDamping>,
  &dobot_msgs_v4__srv__FCSetDamping__get_type_hash,
  &dobot_msgs_v4__srv__FCSetDamping__get_type_description,
  &dobot_msgs_v4__srv__FCSetDamping__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dobot_msgs_v4::srv::FCSetDamping>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_cpp::FCSetDamping_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, dobot_msgs_v4, srv, FCSetDamping)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<dobot_msgs_v4::srv::FCSetDamping>();
}

#ifdef __cplusplus
}
#endif
