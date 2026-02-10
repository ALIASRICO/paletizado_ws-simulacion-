// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/AI.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/ai.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__AI__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__AI__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/ai__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_AI_Request_index
{
public:
  Init_AI_Request_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::AI_Request index(::dobot_msgs_v4::srv::AI_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::AI_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::AI_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_AI_Request_index();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_AI_Response_res
{
public:
  Init_AI_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::AI_Response res(::dobot_msgs_v4::srv::AI_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::AI_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::AI_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_AI_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_AI_Event_response
{
public:
  explicit Init_AI_Event_response(::dobot_msgs_v4::srv::AI_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::AI_Event response(::dobot_msgs_v4::srv::AI_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::AI_Event msg_;
};

class Init_AI_Event_request
{
public:
  explicit Init_AI_Event_request(::dobot_msgs_v4::srv::AI_Event & msg)
  : msg_(msg)
  {}
  Init_AI_Event_response request(::dobot_msgs_v4::srv::AI_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_AI_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::AI_Event msg_;
};

class Init_AI_Event_info
{
public:
  Init_AI_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AI_Event_request info(::dobot_msgs_v4::srv::AI_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_AI_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::AI_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::AI_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_AI_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__AI__BUILDER_HPP_
