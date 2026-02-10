// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/SetBackDistance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/set_back_distance.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SET_BACK_DISTANCE__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SET_BACK_DISTANCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/set_back_distance__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetBackDistance_Request_distance
{
public:
  Init_SetBackDistance_Request_distance()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::SetBackDistance_Request distance(::dobot_msgs_v4::srv::SetBackDistance_Request::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetBackDistance_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetBackDistance_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_SetBackDistance_Request_distance();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetBackDistance_Response_res
{
public:
  Init_SetBackDistance_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::SetBackDistance_Response res(::dobot_msgs_v4::srv::SetBackDistance_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetBackDistance_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetBackDistance_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_SetBackDistance_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetBackDistance_Event_response
{
public:
  explicit Init_SetBackDistance_Event_response(::dobot_msgs_v4::srv::SetBackDistance_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::SetBackDistance_Event response(::dobot_msgs_v4::srv::SetBackDistance_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetBackDistance_Event msg_;
};

class Init_SetBackDistance_Event_request
{
public:
  explicit Init_SetBackDistance_Event_request(::dobot_msgs_v4::srv::SetBackDistance_Event & msg)
  : msg_(msg)
  {}
  Init_SetBackDistance_Event_response request(::dobot_msgs_v4::srv::SetBackDistance_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetBackDistance_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetBackDistance_Event msg_;
};

class Init_SetBackDistance_Event_info
{
public:
  Init_SetBackDistance_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetBackDistance_Event_request info(::dobot_msgs_v4::srv::SetBackDistance_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetBackDistance_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetBackDistance_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetBackDistance_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_SetBackDistance_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SET_BACK_DISTANCE__BUILDER_HPP_
