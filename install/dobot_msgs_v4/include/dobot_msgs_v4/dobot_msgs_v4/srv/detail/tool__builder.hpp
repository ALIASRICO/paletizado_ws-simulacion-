// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/Tool.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/tool.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__TOOL__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__TOOL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/tool__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_Tool_Request_index
{
public:
  Init_Tool_Request_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::Tool_Request index(::dobot_msgs_v4::srv::Tool_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::Tool_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::Tool_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_Tool_Request_index();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_Tool_Response_res
{
public:
  Init_Tool_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::Tool_Response res(::dobot_msgs_v4::srv::Tool_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::Tool_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::Tool_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_Tool_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_Tool_Event_response
{
public:
  explicit Init_Tool_Event_response(::dobot_msgs_v4::srv::Tool_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::Tool_Event response(::dobot_msgs_v4::srv::Tool_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::Tool_Event msg_;
};

class Init_Tool_Event_request
{
public:
  explicit Init_Tool_Event_request(::dobot_msgs_v4::srv::Tool_Event & msg)
  : msg_(msg)
  {}
  Init_Tool_Event_response request(::dobot_msgs_v4::srv::Tool_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Tool_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::Tool_Event msg_;
};

class Init_Tool_Event_info
{
public:
  Init_Tool_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Tool_Event_request info(::dobot_msgs_v4::srv::Tool_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Tool_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::Tool_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::Tool_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_Tool_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__TOOL__BUILDER_HPP_
