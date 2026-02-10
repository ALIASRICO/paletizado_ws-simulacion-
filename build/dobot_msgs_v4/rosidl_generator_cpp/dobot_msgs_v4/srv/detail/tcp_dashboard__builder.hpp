// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/TcpDashboard.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/tcp_dashboard.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__TCP_DASHBOARD__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__TCP_DASHBOARD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/tcp_dashboard__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_TcpDashboard_Request_command
{
public:
  Init_TcpDashboard_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::TcpDashboard_Request command(::dobot_msgs_v4::srv::TcpDashboard_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::TcpDashboard_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::TcpDashboard_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_TcpDashboard_Request_command();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_TcpDashboard_Response_result
{
public:
  Init_TcpDashboard_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::TcpDashboard_Response result(::dobot_msgs_v4::srv::TcpDashboard_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::TcpDashboard_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::TcpDashboard_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_TcpDashboard_Response_result();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_TcpDashboard_Event_response
{
public:
  explicit Init_TcpDashboard_Event_response(::dobot_msgs_v4::srv::TcpDashboard_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::TcpDashboard_Event response(::dobot_msgs_v4::srv::TcpDashboard_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::TcpDashboard_Event msg_;
};

class Init_TcpDashboard_Event_request
{
public:
  explicit Init_TcpDashboard_Event_request(::dobot_msgs_v4::srv::TcpDashboard_Event & msg)
  : msg_(msg)
  {}
  Init_TcpDashboard_Event_response request(::dobot_msgs_v4::srv::TcpDashboard_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_TcpDashboard_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::TcpDashboard_Event msg_;
};

class Init_TcpDashboard_Event_info
{
public:
  Init_TcpDashboard_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TcpDashboard_Event_request info(::dobot_msgs_v4::srv::TcpDashboard_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_TcpDashboard_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::TcpDashboard_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::TcpDashboard_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_TcpDashboard_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__TCP_DASHBOARD__BUILDER_HPP_
