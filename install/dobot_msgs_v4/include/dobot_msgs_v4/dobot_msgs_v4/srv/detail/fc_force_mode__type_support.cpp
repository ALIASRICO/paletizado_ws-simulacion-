// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dobot_msgs_v4:srv/FCForceMode.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dobot_msgs_v4/srv/detail/fc_force_mode__functions.h"
#include "dobot_msgs_v4/srv/detail/fc_force_mode__struct.hpp"
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

void FCForceMode_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::srv::FCForceMode_Request(_init);
}

void FCForceMode_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::srv::FCForceMode_Request *>(message_memory);
  typed_message->~FCForceMode_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FCForceMode_Request_message_member_array[15] = {
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "z",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, z),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, rx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "ry",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, ry),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rz",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, rz),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, fx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fy",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, fy),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fz",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, fz),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "frx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, frx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fry",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, fry),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "frz",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, frz),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reference",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, reference),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "user",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, user),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tool",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Request, tool),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FCForceMode_Request_message_members = {
  "dobot_msgs_v4::srv",  // message namespace
  "FCForceMode_Request",  // message name
  15,  // number of fields
  sizeof(dobot_msgs_v4::srv::FCForceMode_Request),
  false,  // has_any_key_member_
  FCForceMode_Request_message_member_array,  // message members
  FCForceMode_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  FCForceMode_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FCForceMode_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FCForceMode_Request_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__FCForceMode_Request__get_type_hash,
  &dobot_msgs_v4__srv__FCForceMode_Request__get_type_description,
  &dobot_msgs_v4__srv__FCForceMode_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::FCForceMode_Request>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::FCForceMode_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCForceMode_Request)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::FCForceMode_Request_message_type_support_handle;
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
// #include "dobot_msgs_v4/srv/detail/fc_force_mode__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_force_mode__struct.hpp"
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

void FCForceMode_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::srv::FCForceMode_Response(_init);
}

void FCForceMode_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::srv::FCForceMode_Response *>(message_memory);
  typed_message->~FCForceMode_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FCForceMode_Response_message_member_array[1] = {
  {
    "res",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Response, res),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FCForceMode_Response_message_members = {
  "dobot_msgs_v4::srv",  // message namespace
  "FCForceMode_Response",  // message name
  1,  // number of fields
  sizeof(dobot_msgs_v4::srv::FCForceMode_Response),
  false,  // has_any_key_member_
  FCForceMode_Response_message_member_array,  // message members
  FCForceMode_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  FCForceMode_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FCForceMode_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FCForceMode_Response_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__FCForceMode_Response__get_type_hash,
  &dobot_msgs_v4__srv__FCForceMode_Response__get_type_description,
  &dobot_msgs_v4__srv__FCForceMode_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::FCForceMode_Response>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::FCForceMode_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCForceMode_Response)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::FCForceMode_Response_message_type_support_handle;
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
// #include "dobot_msgs_v4/srv/detail/fc_force_mode__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_force_mode__struct.hpp"
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

void FCForceMode_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dobot_msgs_v4::srv::FCForceMode_Event(_init);
}

void FCForceMode_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dobot_msgs_v4::srv::FCForceMode_Event *>(message_memory);
  typed_message->~FCForceMode_Event();
}

size_t size_function__FCForceMode_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dobot_msgs_v4::srv::FCForceMode_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FCForceMode_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dobot_msgs_v4::srv::FCForceMode_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__FCForceMode_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dobot_msgs_v4::srv::FCForceMode_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__FCForceMode_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dobot_msgs_v4::srv::FCForceMode_Request *>(
    get_const_function__FCForceMode_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<dobot_msgs_v4::srv::FCForceMode_Request *>(untyped_value);
  value = item;
}

void assign_function__FCForceMode_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dobot_msgs_v4::srv::FCForceMode_Request *>(
    get_function__FCForceMode_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const dobot_msgs_v4::srv::FCForceMode_Request *>(untyped_value);
  item = value;
}

void resize_function__FCForceMode_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dobot_msgs_v4::srv::FCForceMode_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__FCForceMode_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dobot_msgs_v4::srv::FCForceMode_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FCForceMode_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dobot_msgs_v4::srv::FCForceMode_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__FCForceMode_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dobot_msgs_v4::srv::FCForceMode_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__FCForceMode_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dobot_msgs_v4::srv::FCForceMode_Response *>(
    get_const_function__FCForceMode_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<dobot_msgs_v4::srv::FCForceMode_Response *>(untyped_value);
  value = item;
}

void assign_function__FCForceMode_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dobot_msgs_v4::srv::FCForceMode_Response *>(
    get_function__FCForceMode_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const dobot_msgs_v4::srv::FCForceMode_Response *>(untyped_value);
  item = value;
}

void resize_function__FCForceMode_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dobot_msgs_v4::srv::FCForceMode_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FCForceMode_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Event, info),  // bytes offset in struct
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
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCForceMode_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__FCForceMode_Event__request,  // size() function pointer
    get_const_function__FCForceMode_Event__request,  // get_const(index) function pointer
    get_function__FCForceMode_Event__request,  // get(index) function pointer
    fetch_function__FCForceMode_Event__request,  // fetch(index, &value) function pointer
    assign_function__FCForceMode_Event__request,  // assign(index, value) function pointer
    resize_function__FCForceMode_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCForceMode_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dobot_msgs_v4::srv::FCForceMode_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__FCForceMode_Event__response,  // size() function pointer
    get_const_function__FCForceMode_Event__response,  // get_const(index) function pointer
    get_function__FCForceMode_Event__response,  // get(index) function pointer
    fetch_function__FCForceMode_Event__response,  // fetch(index, &value) function pointer
    assign_function__FCForceMode_Event__response,  // assign(index, value) function pointer
    resize_function__FCForceMode_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FCForceMode_Event_message_members = {
  "dobot_msgs_v4::srv",  // message namespace
  "FCForceMode_Event",  // message name
  3,  // number of fields
  sizeof(dobot_msgs_v4::srv::FCForceMode_Event),
  false,  // has_any_key_member_
  FCForceMode_Event_message_member_array,  // message members
  FCForceMode_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  FCForceMode_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FCForceMode_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FCForceMode_Event_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__FCForceMode_Event__get_type_hash,
  &dobot_msgs_v4__srv__FCForceMode_Event__get_type_description,
  &dobot_msgs_v4__srv__FCForceMode_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dobot_msgs_v4::srv::FCForceMode_Event>()
{
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::FCForceMode_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCForceMode_Event)() {
  return &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::FCForceMode_Event_message_type_support_handle;
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
// #include "dobot_msgs_v4/srv/detail/fc_force_mode__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/fc_force_mode__struct.hpp"
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
static ::rosidl_typesupport_introspection_cpp::ServiceMembers FCForceMode_service_members = {
  "dobot_msgs_v4::srv",  // service namespace
  "FCForceMode",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<dobot_msgs_v4::srv::FCForceMode>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t FCForceMode_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FCForceMode_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCForceMode_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCForceMode_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dobot_msgs_v4::srv::FCForceMode_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<dobot_msgs_v4::srv::FCForceMode>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<dobot_msgs_v4::srv::FCForceMode>,
  &dobot_msgs_v4__srv__FCForceMode__get_type_hash,
  &dobot_msgs_v4__srv__FCForceMode__get_type_description,
  &dobot_msgs_v4__srv__FCForceMode__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace dobot_msgs_v4


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<dobot_msgs_v4::srv::FCForceMode>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::dobot_msgs_v4::srv::rosidl_typesupport_introspection_cpp::FCForceMode_service_type_support_handle;
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
        ::dobot_msgs_v4::srv::FCForceMode_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dobot_msgs_v4::srv::FCForceMode_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::dobot_msgs_v4::srv::FCForceMode_Event
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dobot_msgs_v4, srv, FCForceMode)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<dobot_msgs_v4::srv::FCForceMode>();
}

#ifdef __cplusplus
}
#endif
