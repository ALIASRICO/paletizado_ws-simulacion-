// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dobot_msgs_v4:srv/EnableFTSensor.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/enable_ft_sensor.hpp"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__ENABLE_FT_SENSOR__BUILDER_HPP_
#define DOBOT_MSGS_V4__SRV__DETAIL__ENABLE_FT_SENSOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dobot_msgs_v4/srv/detail/enable_ft_sensor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_EnableFTSensor_Request_status
{
public:
  Init_EnableFTSensor_Request_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::EnableFTSensor_Request status(::dobot_msgs_v4::srv::EnableFTSensor_Request::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::EnableFTSensor_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::EnableFTSensor_Request>()
{
  return dobot_msgs_v4::srv::builder::Init_EnableFTSensor_Request_status();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_EnableFTSensor_Response_res
{
public:
  Init_EnableFTSensor_Response_res()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dobot_msgs_v4::srv::EnableFTSensor_Response res(::dobot_msgs_v4::srv::EnableFTSensor_Response::_res_type arg)
  {
    msg_.res = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::EnableFTSensor_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::EnableFTSensor_Response>()
{
  return dobot_msgs_v4::srv::builder::Init_EnableFTSensor_Response_res();
}

}  // namespace dobot_msgs_v4


namespace dobot_msgs_v4
{

namespace srv
{

namespace builder
{

class Init_EnableFTSensor_Event_response
{
public:
  explicit Init_EnableFTSensor_Event_response(::dobot_msgs_v4::srv::EnableFTSensor_Event & msg)
  : msg_(msg)
  {}
  ::dobot_msgs_v4::srv::EnableFTSensor_Event response(::dobot_msgs_v4::srv::EnableFTSensor_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dobot_msgs_v4::srv::EnableFTSensor_Event msg_;
};

class Init_EnableFTSensor_Event_request
{
public:
  explicit Init_EnableFTSensor_Event_request(::dobot_msgs_v4::srv::EnableFTSensor_Event & msg)
  : msg_(msg)
  {}
  Init_EnableFTSensor_Event_response request(::dobot_msgs_v4::srv::EnableFTSensor_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_EnableFTSensor_Event_response(msg_);
  }

private:
  ::dobot_msgs_v4::srv::EnableFTSensor_Event msg_;
};

class Init_EnableFTSensor_Event_info
{
public:
  Init_EnableFTSensor_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EnableFTSensor_Event_request info(::dobot_msgs_v4::srv::EnableFTSensor_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_EnableFTSensor_Event_request(msg_);
  }

private:
  ::dobot_msgs_v4::srv::EnableFTSensor_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dobot_msgs_v4::srv::EnableFTSensor_Event>()
{
  return dobot_msgs_v4::srv::builder::Init_EnableFTSensor_Event_info();
}

}  // namespace dobot_msgs_v4

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__ENABLE_FT_SENSOR__BUILDER_HPP_
