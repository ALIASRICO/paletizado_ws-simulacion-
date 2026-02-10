// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from dobot_msgs_v4:msg/ToolVectorActual.idl
// generated code does not contain a copyright notice
#ifndef DOBOT_MSGS_V4__MSG__DETAIL__TOOL_VECTOR_ACTUAL__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define DOBOT_MSGS_V4__MSG__DETAIL__TOOL_VECTOR_ACTUAL__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "dobot_msgs_v4/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dobot_msgs_v4/msg/detail/tool_vector_actual__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
bool cdr_serialize_dobot_msgs_v4__msg__ToolVectorActual(
  const dobot_msgs_v4__msg__ToolVectorActual * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
bool cdr_deserialize_dobot_msgs_v4__msg__ToolVectorActual(
  eprosima::fastcdr::Cdr &,
  dobot_msgs_v4__msg__ToolVectorActual * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
size_t get_serialized_size_dobot_msgs_v4__msg__ToolVectorActual(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
size_t max_serialized_size_dobot_msgs_v4__msg__ToolVectorActual(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
bool cdr_serialize_key_dobot_msgs_v4__msg__ToolVectorActual(
  const dobot_msgs_v4__msg__ToolVectorActual * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
size_t get_serialized_size_key_dobot_msgs_v4__msg__ToolVectorActual(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
size_t max_serialized_size_key_dobot_msgs_v4__msg__ToolVectorActual(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dobot_msgs_v4
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dobot_msgs_v4, msg, ToolVectorActual)();

#ifdef __cplusplus
}
#endif

#endif  // DOBOT_MSGS_V4__MSG__DETAIL__TOOL_VECTOR_ACTUAL__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
