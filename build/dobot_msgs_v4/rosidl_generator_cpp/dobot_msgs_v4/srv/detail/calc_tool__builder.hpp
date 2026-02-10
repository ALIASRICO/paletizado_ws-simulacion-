// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/CalcTool.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/calc_tool.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__CALC_TOOL__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__CALC_TOOL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/calc_tool__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_CalcTool_Request_offset
{
public:
  explicit Init_CalcTool_Request_offset(::dobot_msgs_v4::srv::CalcTool_Request & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::CalcTool_Request offset(::dobot_msgs_v4::srv::CalcTool_Request::_offset_type arg)
  {
    msg_.offset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CalcTool_Request msg_;
};

class Init_CalcTool_Request_matrix
{
public:
  explicit Init_CalcTool_Request_matrix(::dobot_msgs_v4::srv::CalcTool_Request & msg)
  : msg_(msg)
  {}
  Init_CalcTool_Request_offset matrix(::dobot_msgs_v4::srv::CalcTool_Request::_matrix_type arg)
  {
    msg_.matrix = std::move(arg);
    return Init_CalcTool_Request_offset(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CalcTool_Request msg_;
};

class Init_CalcTool_Request_index
{
public:
  Init_CalcTool_Request_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalcTool_Request_matrix index(::dobot_msgs_v4::srv::CalcTool_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_CalcTool_Request_matrix(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CalcTool_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::CalcTool_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_CalcTool_Request_index();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_CalcTool_Response_res
{
public:
  Init_CalcTool_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::CalcTool_Response res(::dobot_msgs_v4::srv::CalcTool_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CalcTool_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::CalcTool_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_CalcTool_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_CalcTool_Event_response
{
public:
  explicit Init_CalcTool_Event_response(::dobot_msgs_v4::srv::CalcTool_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::CalcTool_Event response(::dobot_msgs_v4::srv::CalcTool_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CalcTool_Event msg_;
};

class Init_CalcTool_Event_request
{
public:
  explicit Init_CalcTool_Event_request(::dobot_msgs_v4::srv::CalcTool_Event & msg)
  : msg_(msg)
  {}
  Init_CalcTool_Event_response request(::dobot_msgs_v4::srv::CalcTool_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CalcTool_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CalcTool_Event msg_;
};

class Init_CalcTool_Event_info
{
public:
  Init_CalcTool_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalcTool_Event_request info(::dobot_msgs_v4::srv::CalcTool_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CalcTool_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CalcTool_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::CalcTool_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_CalcTool_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__CALC_TOOL__BUILDER_HPP_
