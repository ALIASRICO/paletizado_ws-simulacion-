// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/ToolDOInstant.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/tool_do_instant.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__TOOL_DO_INSTANT__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__TOOL_DO_INSTANT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/tool_do_instant__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_ToolDOInstant_Request_status
{
public:
  explicit Init_ToolDOInstant_Request_status(::dobot_msgs_v4::srv::ToolDOInstant_Request & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::ToolDOInstant_Request status(::dobot_msgs_v4::srv::ToolDOInstant_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDOInstant_Request msg_;
};

class Init_ToolDOInstant_Request_index
{
public:
  Init_ToolDOInstant_Request_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolDOInstant_Request_status index(::dobot_msgs_v4::srv::ToolDOInstant_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_ToolDOInstant_Request_status(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDOInstant_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::ToolDOInstant_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_ToolDOInstant_Request_index();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_ToolDOInstant_Response_res
{
public:
  Init_ToolDOInstant_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::ToolDOInstant_Response res(::dobot_msgs_v4::srv::ToolDOInstant_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDOInstant_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::ToolDOInstant_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_ToolDOInstant_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_ToolDOInstant_Event_response
{
public:
  explicit Init_ToolDOInstant_Event_response(::dobot_msgs_v4::srv::ToolDOInstant_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::ToolDOInstant_Event response(::dobot_msgs_v4::srv::ToolDOInstant_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDOInstant_Event msg_;
};

class Init_ToolDOInstant_Event_request
{
public:
  explicit Init_ToolDOInstant_Event_request(::dobot_msgs_v4::srv::ToolDOInstant_Event & msg)
  : msg_(msg)
  {}
  Init_ToolDOInstant_Event_response request(::dobot_msgs_v4::srv::ToolDOInstant_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ToolDOInstant_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDOInstant_Event msg_;
};

class Init_ToolDOInstant_Event_info
{
public:
  Init_ToolDOInstant_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolDOInstant_Event_request info(::dobot_msgs_v4::srv::ToolDOInstant_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ToolDOInstant_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDOInstant_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::ToolDOInstant_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_ToolDOInstant_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__TOOL_DO_INSTANT__BUILDER_HPP_
