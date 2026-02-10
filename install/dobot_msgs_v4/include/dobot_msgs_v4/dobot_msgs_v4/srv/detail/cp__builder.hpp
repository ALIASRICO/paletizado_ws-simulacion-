// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/CP.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/cp.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__CP__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__CP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/cp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_CP_Request_r
{
public:
  Init_CP_Request_r()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::CP_Request r(::dobot_msgs_v4::srv::CP_Request::_r_type arg)
  {
    msg_.r = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CP_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::CP_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_CP_Request_r();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_CP_Response_res
{
public:
  Init_CP_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::CP_Response res(::dobot_msgs_v4::srv::CP_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CP_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::CP_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_CP_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_CP_Event_response
{
public:
  explicit Init_CP_Event_response(::dobot_msgs_v4::srv::CP_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::CP_Event response(::dobot_msgs_v4::srv::CP_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CP_Event msg_;
};

class Init_CP_Event_request
{
public:
  explicit Init_CP_Event_request(::dobot_msgs_v4::srv::CP_Event & msg)
  : msg_(msg)
  {}
  Init_CP_Event_response request(::dobot_msgs_v4::srv::CP_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CP_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CP_Event msg_;
};

class Init_CP_Event_info
{
public:
  Init_CP_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CP_Event_request info(::dobot_msgs_v4::srv::CP_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CP_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::CP_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::CP_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_CP_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__CP__BUILDER_HPP_
