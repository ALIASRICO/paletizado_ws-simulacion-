// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/SetOutputInt.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/set_output_int.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SET_OUTPUT_INT__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__SET_OUTPUT_INT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/set_output_int__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetOutputInt_Request_value
{
public:
  explicit Init_SetOutputInt_Request_value(::dobot_msgs_v4::srv::SetOutputInt_Request & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::SetOutputInt_Request value(::dobot_msgs_v4::srv::SetOutputInt_Request::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetOutputInt_Request msg_;
};

class Init_SetOutputInt_Request_address
{
public:
  Init_SetOutputInt_Request_address()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetOutputInt_Request_value address(::dobot_msgs_v4::srv::SetOutputInt_Request::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_SetOutputInt_Request_value(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetOutputInt_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetOutputInt_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_SetOutputInt_Request_address();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetOutputInt_Response_res
{
public:
  Init_SetOutputInt_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::SetOutputInt_Response res(::dobot_msgs_v4::srv::SetOutputInt_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetOutputInt_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetOutputInt_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_SetOutputInt_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_SetOutputInt_Event_response
{
public:
  explicit Init_SetOutputInt_Event_response(::dobot_msgs_v4::srv::SetOutputInt_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::SetOutputInt_Event response(::dobot_msgs_v4::srv::SetOutputInt_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetOutputInt_Event msg_;
};

class Init_SetOutputInt_Event_request
{
public:
  explicit Init_SetOutputInt_Event_request(::dobot_msgs_v4::srv::SetOutputInt_Event & msg)
  : msg_(msg)
  {}
  Init_SetOutputInt_Event_response request(::dobot_msgs_v4::srv::SetOutputInt_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetOutputInt_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetOutputInt_Event msg_;
};

class Init_SetOutputInt_Event_info
{
public:
  Init_SetOutputInt_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetOutputInt_Event_request info(::dobot_msgs_v4::srv::SetOutputInt_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetOutputInt_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::SetOutputInt_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::SetOutputInt_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_SetOutputInt_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SET_OUTPUT_INT__BUILDER_HPP_
