// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/DI.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/di.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__DI__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__DI__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/di__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_DI_Request_index
{
public:
  Init_DI_Request_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::DI_Request index(::dobot_msgs_v4::srv::DI_Request::_index_type arg)
  {
    msg_.index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::DI_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::DI_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_DI_Request_index();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_DI_Response_res
{
public:
  explicit Init_DI_Response_res(::dobot_msgs_v4::srv::DI_Response & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::DI_Response res(::dobot_msgs_v4::srv::DI_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::DI_Response msg_;
};

class Init_DI_Response_robot_return
{
public:
  Init_DI_Response_robot_return()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DI_Response_res robot_return(::dobot_msgs_v4::srv::DI_Response::_robot_return_type arg)
  {
    msg_.robot_return = std::move(arg);
    return Init_DI_Response_res(msg_);
  }

private:
  ::dobot_msgs_v4::srv::DI_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::DI_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_DI_Response_robot_return();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_DI_Event_response
{
public:
  explicit Init_DI_Event_response(::dobot_msgs_v4::srv::DI_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::DI_Event response(::dobot_msgs_v4::srv::DI_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::DI_Event msg_;
};

class Init_DI_Event_request
{
public:
  explicit Init_DI_Event_request(::dobot_msgs_v4::srv::DI_Event & msg)
  : msg_(msg)
  {}
  Init_DI_Event_response request(::dobot_msgs_v4::srv::DI_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_DI_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::DI_Event msg_;
};

class Init_DI_Event_info
{
public:
  Init_DI_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DI_Event_request info(::dobot_msgs_v4::srv::DI_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_DI_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::DI_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::DI_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_DI_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__DI__BUILDER_HPP_
