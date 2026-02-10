// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v4:srv/RelMovJUser.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dobot_msgs_v4/srv/detail/rel_mov_j_user__functions.h"
#include "dobot_msgs_v4/srv/detail/rel_mov_j_user__struct.hpp"
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

void RelMovJUser_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::srv::RelMovJUser_Request(_init);
}

void RelMovJUser_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::srv::RelMovJUser_Request *>(message_memory);
  typed_message->~RelMovJUser_Request();
}

size_t size_function__RelMovJUser_Request__param_value(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RelMovJUser_Request__param_value(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__RelMovJUser_Request__param_value(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__RelMovJUser_Request__param_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__RelMovJUser_Request__param_value(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__RelMovJUser_Request__param_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__RelMovJUser_Request__param_value(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__RelMovJUser_Request__param_value(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RelMovJUser_Request_message_member_array[7] = {
  {
    "a",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Request, a),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "b",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Request, b),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "c",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Request, c),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "d",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Request, d),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "e",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Request, e),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "f",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Request, f),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "param_value",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Request, param_value),  // bytes offset in struct
    nullptr,  // default value
    size_function__RelMovJUser_Request__param_value,  // size() function pointer
    get_const_function__RelMovJUser_Request__param_value,  // get_const(index) function pointer
    get_function__RelMovJUser_Request__param_value,  // get(index) function pointer
    fetch_function__RelMovJUser_Request__param_value,  // fetch(index, &value) function pointer
    assign_function__RelMovJUser_Request__param_value,  // assign(index, value) function pointer
    resize_function__RelMovJUser_Request__param_value  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RelMovJUser_Request_message_members = {
  "dobot_msgs_v4::srv",  // message namespace
  "RelMovJUser_Request",  // message name
  7,  // number of fields
  sizeof(dobot_msgs_v4::srv::RelMovJUser_Request),
  false,  // has_any_key_member_
  RelMovJUser_Request_message_member_array,  // message members
  RelMovJUser_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  RelMovJUser_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RelMovJUser_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RelMovJUser_Request_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__RelMovJUser_Request__get_type_hash,
  &dobot_msgs_v4__srv__RelMovJUser_Request__get_type_description,
  &dobot_msgs_v4__srv__RelMovJUser_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::RelMovJUser_Request>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::RelMovJUser_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, RelMovJUser_Request)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::RelMovJUser_Request_message_type_support_handle;
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
// #include "dobot_msgs_v4/srv/detail/rel_mov_j_user__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/rel_mov_j_user__struct.hpp"
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

void RelMovJUser_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::srv::RelMovJUser_Response(_init);
}

void RelMovJUser_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::srv::RelMovJUser_Response *>(message_memory);
  typed_message->~RelMovJUser_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RelMovJUser_Response_message_member_array[1] = {
  {
    "res",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Response, res),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RelMovJUser_Response_message_members = {
  "dobot_msgs_v4::srv",  // message namespace
  "RelMovJUser_Response",  // message name
  1,  // number of fields
  sizeof(dobot_msgs_v4::srv::RelMovJUser_Response),
  false,  // has_any_key_member_
  RelMovJUser_Response_message_member_array,  // message members
  RelMovJUser_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  RelMovJUser_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RelMovJUser_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RelMovJUser_Response_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__RelMovJUser_Response__get_type_hash,
  &dobot_msgs_v4__srv__RelMovJUser_Response__get_type_description,
  &dobot_msgs_v4__srv__RelMovJUser_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::RelMovJUser_Response>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::RelMovJUser_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, RelMovJUser_Response)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::RelMovJUser_Response_message_type_support_handle;
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
// #include "dobot_msgs_v4/srv/detail/rel_mov_j_user__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/rel_mov_j_user__struct.hpp"
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

void RelMovJUser_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::srv::RelMovJUser_Event(_init);
}

void RelMovJUser_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::srv::RelMovJUser_Event *>(message_memory);
  typed_message->~RelMovJUser_Event();
}

size_t size_function__RelMovJUser_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dobot_msgs_v4::srv::RelMovJUser_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RelMovJUser_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dobot_msgs_v4::srv::RelMovJUser_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__RelMovJUser_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dobot_msgs_v4::srv::RelMovJUser_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__RelMovJUser_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dobot_msgs_v4::srv::RelMovJUser_Request *>(
    get_const_function__RelMovJUser_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<dobot_msgs_v4::srv::RelMovJUser_Request *>(untyped_value);
  value = item;
}

void assign_function__RelMovJUser_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dobot_msgs_v4::srv::RelMovJUser_Request *>(
    get_function__RelMovJUser_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const dobot_msgs_v4::srv::RelMovJUser_Request *>(untyped_value);
  item = value;
}

void resize_function__RelMovJUser_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dobot_msgs_v4::srv::RelMovJUser_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RelMovJUser_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dobot_msgs_v4::srv::RelMovJUser_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RelMovJUser_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dobot_msgs_v4::srv::RelMovJUser_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__RelMovJUser_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dobot_msgs_v4::srv::RelMovJUser_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__RelMovJUser_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dobot_msgs_v4::srv::RelMovJUser_Response *>(
    get_const_function__RelMovJUser_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<dobot_msgs_v4::srv::RelMovJUser_Response *>(untyped_value);
  value = item;
}

void assign_function__RelMovJUser_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dobot_msgs_v4::srv::RelMovJUser_Response *>(
    get_function__RelMovJUser_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const dobot_msgs_v4::srv::RelMovJUser_Response *>(untyped_value);
  item = value;
}

void resize_function__RelMovJUser_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dobot_msgs_v4::srv::RelMovJUser_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RelMovJUser_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Event, info),  // bytes offset in struct
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
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::RelMovJUser_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__RelMovJUser_Event__request,  // size() function pointer
    get_const_function__RelMovJUser_Event__request,  // get_const(index) function pointer
    get_function__RelMovJUser_Event__request,  // get(index) function pointer
    fetch_function__RelMovJUser_Event__request,  // fetch(index, &value) function pointer
    assign_function__RelMovJUser_Event__request,  // assign(index, value) function pointer
    resize_function__RelMovJUser_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::RelMovJUser_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dobot_msgs_v4::srv::RelMovJUser_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__RelMovJUser_Event__response,  // size() function pointer
    get_const_function__RelMovJUser_Event__response,  // get_const(index) function pointer
    get_function__RelMovJUser_Event__response,  // get(index) function pointer
    fetch_function__RelMovJUser_Event__response,  // fetch(index, &value) function pointer
    assign_function__RelMovJUser_Event__response,  // assign(index, value) function pointer
    resize_function__RelMovJUser_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RelMovJUser_Event_message_members = {
  "dobot_msgs_v4::srv",  // message namespace
  "RelMovJUser_Event",  // message name
  3,  // number of fields
  sizeof(dobot_msgs_v4::srv::RelMovJUser_Event),
  false,  // has_any_key_member_
  RelMovJUser_Event_message_member_array,  // message members
  RelMovJUser_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  RelMovJUser_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RelMovJUser_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RelMovJUser_Event_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__RelMovJUser_Event__get_type_hash,
  &dobot_msgs_v4__srv__RelMovJUser_Event__get_type_description,
  &dobot_msgs_v4__srv__RelMovJUser_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::RelMovJUser_Event>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::RelMovJUser_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, RelMovJUser_Event)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::RelMovJUser_Event_message_type_support_handle;
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
// #include "dobot_msgs_v4/srv/detail/rel_mov_j_user__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/rel_mov_j_user__struct.hpp"
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
static ::rosidl_typesupport_introspection_cpp::ServiceMembers RelMovJUser_service_members = {
  "dobot_msgs_v4::srv",  // service namespace
  "RelMovJUser",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<dobot_msgs_v4::srv::RelMovJUser>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t RelMovJUser_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RelMovJUser_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::RelMovJUser_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::RelMovJUser_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::RelMovJUser_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<dobot_msgs_v4::srv::RelMovJUser>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<dobot_msgs_v4::srv::RelMovJUser>,
  &dobot_msgs_v4__srv__RelMovJUser__get_type_hash,
  &dobot_msgs_v4__srv__RelMovJUser__get_type_description,
  &dobot_msgs_v4__srv__RelMovJUser__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dobot_msgs_v4::srv::RelMovJUser>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::RelMovJUser_service_type_support_handle;
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
        ::dobot_msgs_v4::srv::RelMovJUser_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dobot_msgs_v4::srv::RelMovJUser_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dobot_msgs_v4::srv::RelMovJUser_Event
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, RelMovJUser)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<dobot_msgs_v4::srv::RelMovJUser>();
}

#ifdef __cplusplus
}
#endif
