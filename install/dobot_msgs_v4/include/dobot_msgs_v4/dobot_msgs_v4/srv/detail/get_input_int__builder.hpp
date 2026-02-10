// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/GetInputInt.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/get_input_int.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__GET_INPUT_INT__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__GET_INPUT_INT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/get_input_int__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_GetInputInt_Request_address
{
public:
  Init_GetInputInt_Request_address()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::GetInputInt_Request address(::dobot_msgs_v4::srv::GetInputInt_Request::_address_type arg)
  {
    msg_.address = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::GetInputInt_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::GetInputInt_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_GetInputInt_Request_address();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_GetInputInt_Response_res
{
public:
  explicit Init_GetInputInt_Response_res(::dobot_msgs_v4::srv::GetInputInt_Response & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::GetInputInt_Response res(::dobot_msgs_v4::srv::GetInputInt_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::GetInputInt_Response msg_;
};

class Init_GetInputInt_Response_robot_return
{
public:
  Init_GetInputInt_Response_robot_return()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetInputInt_Response_res robot_return(::dobot_msgs_v4::srv::GetInputInt_Response::_robot_return_type arg)
  {
    msg_.robot_return = std::move(arg);
    return Init_GetInputInt_Response_res(msg_);
  }

private:
  ::dobot_msgs_v4::srv::GetInputInt_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::GetInputInt_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_GetInputInt_Response_robot_return();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_GetInputInt_Event_response
{
public:
  explicit Init_GetInputInt_Event_response(::dobot_msgs_v4::srv::GetInputInt_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::GetInputInt_Event response(::dobot_msgs_v4::srv::GetInputInt_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::GetInputInt_Event msg_;
};

class Init_GetInputInt_Event_request
{
public:
  explicit Init_GetInputInt_Event_request(::dobot_msgs_v4::srv::GetInputInt_Event & msg)
  : msg_(msg)
  {}
  Init_GetInputInt_Event_response request(::dobot_msgs_v4::srv::GetInputInt_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetInputInt_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::GetInputInt_Event msg_;
};

class Init_GetInputInt_Event_info
{
public:
  Init_GetInputInt_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetInputInt_Event_request info(::dobot_msgs_v4::srv::GetInputInt_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetInputInt_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::GetInputInt_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::GetInputInt_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_GetInputInt_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__GET_INPUT_INT__BUILDER_HPP_
