// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dobot_msgs_v4:srv/PositiveKin.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dobot_msgs_v4/srv/detail/positive_kin__rosidl_typesupport_introspection_c.h"
#include "dobot_msgs_v4/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dobot_msgs_v4/srv/detail/positive_kin__functions.h"
#include "dobot_msgs_v4/srv/detail/positive_kin__struct.h"


// Include directives for member types
// Member `user`
// Member `tool`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dobot_msgs_v4__srv__PositiveKin_Request__init(message_memory);
}

void dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_fini_function(void * message_memory)
{
  dobot_msgs_v4__srv__PositiveKin_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_message_member_array[8] = {
  {
    "j1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Request, j1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "j2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Request, j2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "j3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Request, j3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "j4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Request, j4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "j5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Request, j5),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "j6",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Request, j6),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "user",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Request, user),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tool",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Request, tool),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_message_members = {
  "dobot_msgs_v4__srv",  // message namespace
  "PositiveKin_Request",  // message name
  8,  // number of fields
  sizeof(dobot_msgs_v4__srv__PositiveKin_Request),
  false,  // has_any_key_member_
  dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_message_member_array,  // message members
  dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_message_type_support_handle = {
  0,
  &dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__PositiveKin_Request__get_type_hash,
  &dobot_msgs_v4__srv__PositiveKin_Request__get_type_description,
  &dobot_msgs_v4__srv__PositiveKin_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dobot_msgs_v4
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Request)() {
  if (!dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_message_type_support_handle.typesupport_identifier) {
    dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dobot_msgs_v4/srv/detail/positive_kin__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dobot_msgs_v4/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/positive_kin__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/positive_kin__struct.h"


// Include directives for member types
// Member `robot_return`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dobot_msgs_v4__srv__PositiveKin_Response__init(message_memory);
}

void dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_fini_function(void * message_memory)
{
  dobot_msgs_v4__srv__PositiveKin_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_message_member_array[2] = {
  {
    "robot_return",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Response, robot_return),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "res",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Response, res),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_message_members = {
  "dobot_msgs_v4__srv",  // message namespace
  "PositiveKin_Response",  // message name
  2,  // number of fields
  sizeof(dobot_msgs_v4__srv__PositiveKin_Response),
  false,  // has_any_key_member_
  dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_message_member_array,  // message members
  dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_message_type_support_handle = {
  0,
  &dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__PositiveKin_Response__get_type_hash,
  &dobot_msgs_v4__srv__PositiveKin_Response__get_type_description,
  &dobot_msgs_v4__srv__PositiveKin_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dobot_msgs_v4
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Response)() {
  if (!dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_message_type_support_handle.typesupport_identifier) {
    dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dobot_msgs_v4/srv/detail/positive_kin__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dobot_msgs_v4/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/positive_kin__functions.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/positive_kin__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "dobot_msgs_v4/srv/positive_kin.h"
// Member `request`
// Member `response`
// already included above
// #include "dobot_msgs_v4/srv/detail/positive_kin__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dobot_msgs_v4__srv__PositiveKin_Event__init(message_memory);
}

void dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_fini_function(void * message_memory)
{
  dobot_msgs_v4__srv__PositiveKin_Event__fini(message_memory);
}

size_t dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__size_function__PositiveKin_Event__request(
  const void * untyped_member)
{
  const dobot_msgs_v4__srv__PositiveKin_Request__Sequence * member =
    (const dobot_msgs_v4__srv__PositiveKin_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_const_function__PositiveKin_Event__request(
  const void * untyped_member, size_t index)
{
  const dobot_msgs_v4__srv__PositiveKin_Request__Sequence * member =
    (const dobot_msgs_v4__srv__PositiveKin_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_function__PositiveKin_Event__request(
  void * untyped_member, size_t index)
{
  dobot_msgs_v4__srv__PositiveKin_Request__Sequence * member =
    (dobot_msgs_v4__srv__PositiveKin_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__fetch_function__PositiveKin_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dobot_msgs_v4__srv__PositiveKin_Request * item =
    ((const dobot_msgs_v4__srv__PositiveKin_Request *)
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_const_function__PositiveKin_Event__request(untyped_member, index));
  dobot_msgs_v4__srv__PositiveKin_Request * value =
    (dobot_msgs_v4__srv__PositiveKin_Request *)(untyped_value);
  *value = *item;
}

void dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__assign_function__PositiveKin_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dobot_msgs_v4__srv__PositiveKin_Request * item =
    ((dobot_msgs_v4__srv__PositiveKin_Request *)
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_function__PositiveKin_Event__request(untyped_member, index));
  const dobot_msgs_v4__srv__PositiveKin_Request * value =
    (const dobot_msgs_v4__srv__PositiveKin_Request *)(untyped_value);
  *item = *value;
}

bool dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__resize_function__PositiveKin_Event__request(
  void * untyped_member, size_t size)
{
  dobot_msgs_v4__srv__PositiveKin_Request__Sequence * member =
    (dobot_msgs_v4__srv__PositiveKin_Request__Sequence *)(untyped_member);
  dobot_msgs_v4__srv__PositiveKin_Request__Sequence__fini(member);
  return dobot_msgs_v4__srv__PositiveKin_Request__Sequence__init(member, size);
}

size_t dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__size_function__PositiveKin_Event__response(
  const void * untyped_member)
{
  const dobot_msgs_v4__srv__PositiveKin_Response__Sequence * member =
    (const dobot_msgs_v4__srv__PositiveKin_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_const_function__PositiveKin_Event__response(
  const void * untyped_member, size_t index)
{
  const dobot_msgs_v4__srv__PositiveKin_Response__Sequence * member =
    (const dobot_msgs_v4__srv__PositiveKin_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_function__PositiveKin_Event__response(
  void * untyped_member, size_t index)
{
  dobot_msgs_v4__srv__PositiveKin_Response__Sequence * member =
    (dobot_msgs_v4__srv__PositiveKin_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__fetch_function__PositiveKin_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dobot_msgs_v4__srv__PositiveKin_Response * item =
    ((const dobot_msgs_v4__srv__PositiveKin_Response *)
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_const_function__PositiveKin_Event__response(untyped_member, index));
  dobot_msgs_v4__srv__PositiveKin_Response * value =
    (dobot_msgs_v4__srv__PositiveKin_Response *)(untyped_value);
  *value = *item;
}

void dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__assign_function__PositiveKin_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dobot_msgs_v4__srv__PositiveKin_Response * item =
    ((dobot_msgs_v4__srv__PositiveKin_Response *)
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_function__PositiveKin_Event__response(untyped_member, index));
  const dobot_msgs_v4__srv__PositiveKin_Response * value =
    (const dobot_msgs_v4__srv__PositiveKin_Response *)(untyped_value);
  *item = *value;
}

bool dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__resize_function__PositiveKin_Event__response(
  void * untyped_member, size_t size)
{
  dobot_msgs_v4__srv__PositiveKin_Response__Sequence * member =
    (dobot_msgs_v4__srv__PositiveKin_Response__Sequence *)(untyped_member);
  dobot_msgs_v4__srv__PositiveKin_Response__Sequence__fini(member);
  return dobot_msgs_v4__srv__PositiveKin_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Event, request),  // bytes offset in struct
    NULL,  // default value
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__size_function__PositiveKin_Event__request,  // size() function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_const_function__PositiveKin_Event__request,  // get_const(index) function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_function__PositiveKin_Event__request,  // get(index) function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__fetch_function__PositiveKin_Event__request,  // fetch(index, &value) function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__assign_function__PositiveKin_Event__request,  // assign(index, value) function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__resize_function__PositiveKin_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(dobot_msgs_v4__srv__PositiveKin_Event, response),  // bytes offset in struct
    NULL,  // default value
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__size_function__PositiveKin_Event__response,  // size() function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_const_function__PositiveKin_Event__response,  // get_const(index) function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__get_function__PositiveKin_Event__response,  // get(index) function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__fetch_function__PositiveKin_Event__response,  // fetch(index, &value) function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__assign_function__PositiveKin_Event__response,  // assign(index, value) function pointer
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__resize_function__PositiveKin_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_members = {
  "dobot_msgs_v4__srv",  // message namespace
  "PositiveKin_Event",  // message name
  3,  // number of fields
  sizeof(dobot_msgs_v4__srv__PositiveKin_Event),
  false,  // has_any_key_member_
  dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_member_array,  // message members
  dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_type_support_handle = {
  0,
  &dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_members,
  get_message_typesupport_handle_function,
  &dobot_msgs_v4__srv__PositiveKin_Event__get_type_hash,
  &dobot_msgs_v4__srv__PositiveKin_Event__get_type_description,
  &dobot_msgs_v4__srv__PositiveKin_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dobot_msgs_v4
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Event)() {
  dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Request)();
  dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Response)();
  if (!dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_type_support_handle.typesupport_identifier) {
    dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dobot_msgs_v4/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "dobot_msgs_v4/srv/detail/positive_kin__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_service_members = {
  "dobot_msgs_v4__srv",  // service namespace
  "PositiveKin",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_Request_message_type_support_handle,
  NULL,  // response message
  // dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_Response_message_type_support_handle
  NULL  // event_message
  // dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_Response_message_type_support_handle
};


static rosidl_service_type_support_t dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_service_type_support_handle = {
  0,
  &dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_service_members,
  get_service_typesupport_handle_function,
  &dobot_msgs_v4__srv__PositiveKin_Request__rosidl_typesupport_introspection_c__PositiveKin_Request_message_type_support_handle,
  &dobot_msgs_v4__srv__PositiveKin_Response__rosidl_typesupport_introspection_c__PositiveKin_Response_message_type_support_handle,
  &dobot_msgs_v4__srv__PositiveKin_Event__rosidl_typesupport_introspection_c__PositiveKin_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dobot_msgs_v4,
    srv,
    PositiveKin
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    dobot_msgs_v4,
    srv,
    PositiveKin
  ),
  &dobot_msgs_v4__srv__PositiveKin__get_type_hash,
  &dobot_msgs_v4__srv__PositiveKin__get_type_description,
  &dobot_msgs_v4__srv__PositiveKin__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dobot_msgs_v4
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin)(void) {
  if (!dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_service_type_support_handle.typesupport_identifier) {
    dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dobot_msgs_v4, srv, PositiveKin_Event)()->data;
  }

  return &dobot_msgs_v4__srv__detail__positive_kin__rosidl_typesupport_introspection_c__PositiveKin_service_type_support_handle;
}
