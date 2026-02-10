// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v4:srv/GetCoils.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dobot_msgs_v4/srv/detail/get_coils__functions.h"
#include "dobot_msgs_v4/srv/detail/get_coils__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetCoils_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::srv::GetCoils_Request(_init);
}

void GetCoils_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::srv::GetCoils_Request *>(message_memory);
  typed_message->~GetCoils_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetCoils_Request_message_member_array[3] = {
  {
    "index",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::GetCoils_Request, index),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "addr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::GetCoils_Request, addr),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "count",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::GetCoils_Request, count),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetCoils_Request_message_members = {
  "dobot_msgs_v4::srv",  // message namespace
  "GetCoils_Request",  // message name
  3,  // number of fields
  sizeof(dobot_msgs_v4::srv::GetCoils_Request),
  false,  // has_any_key_member_
  GetCoils_Request_message_member_array,  // message members
  GetCoils_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetCoils_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetCoils_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetCoils_Request_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__GetCoils_Request__get_type_hash,
  &dobot_msgs_v4__srv__GetCoils_Request__get_type_description,
  &dobot_msgs_v4__srv__GetCoils_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::GetCoils_Request>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::GetCoils_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, GetCoils_Request)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::GetCoils_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/get_coils__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/get_coils__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetCoils_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::srv::GetCoils_Response(_init);
}

void GetCoils_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::srv::GetCoils_Response *>(message_memory);
  typed_message->~GetCoils_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetCoils_Response_message_member_array[2] = {
  {
    "robot_return",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::GetCoils_Response, robot_return),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "res",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::GetCoils_Response, res),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetCoils_Response_message_members = {
  "dobot_msgs_v4::srv",  // message namespace
  "GetCoils_Response",  // message name
  2,  // number of fields
  sizeof(dobot_msgs_v4::srv::GetCoils_Response),
  false,  // has_any_key_member_
  GetCoils_Response_message_member_array,  // message members
  GetCoils_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetCoils_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetCoils_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetCoils_Response_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__GetCoils_Response__get_type_hash,
  &dobot_msgs_v4__srv__GetCoils_Response__get_type_description,
  &dobot_msgs_v4__srv__GetCoils_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::GetCoils_Response>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::GetCoils_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, GetCoils_Response)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::GetCoils_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/get_coils__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/get_coils__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetCoils_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::srv::GetCoils_Event(_init);
}

void GetCoils_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::srv::GetCoils_Event *>(message_memory);
  typed_message->~GetCoils_Event();
}

size_t size_function__GetCoils_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dobot_msgs_v4::srv::GetCoils_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetCoils_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dobot_msgs_v4::srv::GetCoils_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__GetCoils_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dobot_msgs_v4::srv::GetCoils_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetCoils_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dobot_msgs_v4::srv::GetCoils_Request *>(
    get_const_function__GetCoils_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<dobot_msgs_v4::srv::GetCoils_Request *>(untyped_value);
  value = item;
}

void assign_function__GetCoils_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dobot_msgs_v4::srv::GetCoils_Request *>(
    get_function__GetCoils_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const dobot_msgs_v4::srv::GetCoils_Request *>(untyped_value);
  item = value;
}

void resize_function__GetCoils_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dobot_msgs_v4::srv::GetCoils_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetCoils_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dobot_msgs_v4::srv::GetCoils_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetCoils_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dobot_msgs_v4::srv::GetCoils_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__GetCoils_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dobot_msgs_v4::srv::GetCoils_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetCoils_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dobot_msgs_v4::srv::GetCoils_Response *>(
    get_const_function__GetCoils_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<dobot_msgs_v4::srv::GetCoils_Response *>(untyped_value);
  value = item;
}

void assign_function__GetCoils_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dobot_msgs_v4::srv::GetCoils_Response *>(
    get_function__GetCoils_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const dobot_msgs_v4::srv::GetCoils_Response *>(untyped_value);
  item = value;
}

void resize_function__GetCoils_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dobot_msgs_v4::srv::GetCoils_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetCoils_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::GetCoils_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::GetCoils_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dobot_msgs_v4::srv::GetCoils_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetCoils_Event__request,  // size() function pointer
    get_const_function__GetCoils_Event__request,  // get_const(index) function pointer
    get_function__GetCoils_Event__request,  // get(index) function pointer
    fetch_function__GetCoils_Event__request,  // fetch(index, &value) function pointer
    assign_function__GetCoils_Event__request,  // assign(index, value) function pointer
    resize_function__GetCoils_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::GetCoils_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dobot_msgs_v4::srv::GetCoils_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetCoils_Event__response,  // size() function pointer
    get_const_function__GetCoils_Event__response,  // get_const(index) function pointer
    get_function__GetCoils_Event__response,  // get(index) function pointer
    fetch_function__GetCoils_Event__response,  // fetch(index, &value) function pointer
    assign_function__GetCoils_Event__response,  // assign(index, value) function pointer
    resize_function__GetCoils_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetCoils_Event_message_members = {
  "dobot_msgs_v4::srv",  // message namespace
  "GetCoils_Event",  // message name
  3,  // number of fields
  sizeof(dobot_msgs_v4::srv::GetCoils_Event),
  false,  // has_any_key_member_
  GetCoils_Event_message_member_array,  // message members
  GetCoils_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  GetCoils_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetCoils_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetCoils_Event_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__GetCoils_Event__get_type_hash,
  &dobot_msgs_v4__srv__GetCoils_Event__get_type_description,
  &dobot_msgs_v4__srv__GetCoils_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::GetCoils_Event>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::GetCoils_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, GetCoils_Event)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::GetCoils_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/get_coils__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/get_coils__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace dobot_msgs_v4
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GetCoils_service_members = {
  "dobot_msgs_v4::srv",  // service namespace
  "GetCoils",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<dobot_msgs_v4::srv::GetCoils>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t GetCoils_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetCoils_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::GetCoils_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::GetCoils_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::GetCoils_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<dobot_msgs_v4::srv::GetCoils>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<dobot_msgs_v4::srv::GetCoils>,
  &dobot_msgs_v4__srv__GetCoils__get_type_hash,
  &dobot_msgs_v4__srv__GetCoils__get_type_description,
  &dobot_msgs_v4__srv__GetCoils__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dobot_msgs_v4::srv::GetCoils>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::GetCoils_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dobot_msgs_v4::srv::GetCoils_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dobot_msgs_v4::srv::GetCoils_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dobot_msgs_v4::srv::GetCoils_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, GetCoils)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<dobot_msgs_v4::srv::GetCoils>();
}

#ifdef __cplusplus
}
#endif
