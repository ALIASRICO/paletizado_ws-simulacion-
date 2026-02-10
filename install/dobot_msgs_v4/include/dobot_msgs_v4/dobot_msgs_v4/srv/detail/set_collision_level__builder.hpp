// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/SetCollisionLevel.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/set_collision_level.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SET_COLLISION_LEVEL__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SET_COLLISION_LEVEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/set_collision_level__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetCollisionLevel_Request_level
{
public:
  Init_SetCollisionLevel_Request_level()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::SetCollisionLevel_Request level(::dobot_msgs_v4::srv::SetCollisionLevel_Request::_level_type arg)
  {
    msg_.level = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetCollisionLevel_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetCollisionLevel_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_SetCollisionLevel_Request_level();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetCollisionLevel_Response_res
{
public:
  Init_SetCollisionLevel_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::SetCollisionLevel_Response res(::dobot_msgs_v4::srv::SetCollisionLevel_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetCollisionLevel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetCollisionLevel_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_SetCollisionLevel_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetCollisionLevel_Event_response
{
public:
  explicit Init_SetCollisionLevel_Event_response(::dobot_msgs_v4::srv::SetCollisionLevel_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::SetCollisionLevel_Event response(::dobot_msgs_v4::srv::SetCollisionLevel_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetCollisionLevel_Event msg_;
};

class Init_SetCollisionLevel_Event_request
{
public:
  explicit Init_SetCollisionLevel_Event_request(::dobot_msgs_v4::srv::SetCollisionLevel_Event & msg)
  : msg_(msg)
  {}
  Init_SetCollisionLevel_Event_response request(::dobot_msgs_v4::srv::SetCollisionLevel_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetCollisionLevel_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetCollisionLevel_Event msg_;
};

class Init_SetCollisionLevel_Event_info
{
public:
  Init_SetCollisionLevel_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetCollisionLevel_Event_request info(::dobot_msgs_v4::srv::SetCollisionLevel_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetCollisionLevel_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetCollisionLevel_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetCollisionLevel_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_SetCollisionLevel_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SET_COLLISION_LEVEL__BUILDER_HPP_
