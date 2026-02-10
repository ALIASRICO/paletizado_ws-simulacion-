// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dobot_msgs_v4:srv/Continue.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/continue.h"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__CONTINUE__STRUCT_H_
#define DOBOT_MSGS_V4__SRV__DETAIL__CONTINUE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Continue in the package dobot_msgs_v4.
typedef struct dobot_msgs_v4__srv__Continue_Request
{
  uint8_t structure_needs_at_least_one_member;
} dobot_msgs_v4__srv__Continue_Request;

// Struct for a sequence of dobot_msgs_v4__srv__Continue_Request.
typedef struct dobot_msgs_v4__srv__Continue_Request__Sequence
{
  dobot_msgs_v4__srv__Continue_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dobot_msgs_v4__srv__Continue_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/Continue in the package dobot_msgs_v4.
typedef struct dobot_msgs_v4__srv__Continue_Response
{
  int32_t res;
} dobot_msgs_v4__srv__Continue_Response;

// Struct for a sequence of dobot_msgs_v4__srv__Continue_Response.
typedef struct dobot_msgs_v4__srv__Continue_Response__Sequence
{
  dobot_msgs_v4__srv__Continue_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dobot_msgs_v4__srv__Continue_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  dobot_msgs_v4__srv__Continue_Event__request__MAX_SIZE = 1
};
// response
enum
{
  dobot_msgs_v4__srv__Continue_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Continue in the package dobot_msgs_v4.
typedef struct dobot_msgs_v4__srv__Continue_Event
{
  service_msgs__msg__ServiceEventInfo info;
  dobot_msgs_v4__srv__Continue_Request__Sequence request;
  dobot_msgs_v4__srv__Continue_Response__Sequence response;
} dobot_msgs_v4__srv__Continue_Event;

// Struct for a sequence of dobot_msgs_v4__srv__Continue_Event.
typedef struct dobot_msgs_v4__srv__Continue_Event__Sequence
{
  dobot_msgs_v4__srv__Continue_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dobot_msgs_v4__srv__Continue_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__CONTINUE__STRUCT_H_
