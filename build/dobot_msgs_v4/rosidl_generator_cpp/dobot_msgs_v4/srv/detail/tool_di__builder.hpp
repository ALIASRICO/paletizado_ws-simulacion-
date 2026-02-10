// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/ToolDI.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/tool_di.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__TOOL_DI__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__TOOL_DI__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/tool_di__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_ToolDI_Request_index
{
public:
  Init_ToolDI_Request_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::ToolDI_Request index(::dobot_msgs_v4::srv::ToolDI_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDI_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::ToolDI_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_ToolDI_Request_index();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_ToolDI_Response_res
{
public:
  Init_ToolDI_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::ToolDI_Response res(::dobot_msgs_v4::srv::ToolDI_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDI_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::ToolDI_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_ToolDI_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_ToolDI_Event_response
{
public:
  explicit Init_ToolDI_Event_response(::dobot_msgs_v4::srv::ToolDI_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::ToolDI_Event response(::dobot_msgs_v4::srv::ToolDI_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDI_Event msg_;
};

class Init_ToolDI_Event_request
{
public:
  explicit Init_ToolDI_Event_request(::dobot_msgs_v4::srv::ToolDI_Event & msg)
  : msg_(msg)
  {}
  Init_ToolDI_Event_response request(::dobot_msgs_v4::srv::ToolDI_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ToolDI_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDI_Event msg_;
};

class Init_ToolDI_Event_info
{
public:
  Init_ToolDI_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolDI_Event_request info(::dobot_msgs_v4::srv::ToolDI_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ToolDI_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::ToolDI_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::ToolDI_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_ToolDI_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__TOOL_DI__BUILDER_HPP_
