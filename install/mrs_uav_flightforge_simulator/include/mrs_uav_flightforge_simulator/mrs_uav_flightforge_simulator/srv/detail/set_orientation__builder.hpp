// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mrs_uav_flightforge_simulator:srv/SetOrientation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mrs_uav_flightforge_simulator/srv/set_orientation.hpp"


#ifndef MRS_UAV_FLIGHTFORGE_SIMULATOR__SRV__DETAIL__SET_ORIENTATION__BUILDER_HPP_
#define MRS_UAV_FLIGHTFORGE_SIMULATOR__SRV__DETAIL__SET_ORIENTATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mrs_uav_flightforge_simulator
{

namespace srv
{

namespace builder
{

class Init_SetOrientation_Request_quaternion
{
public:
  Init_SetOrientation_Request_quaternion()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mrs_uav_flightforge_simulator::srv::SetOrientation_Request quaternion(::mrs_uav_flightforge_simulator::srv::SetOrientation_Request::_quaternion_type arg)
  {
    msg_.quaternion = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mrs_uav_flightforge_simulator::srv::SetOrientation_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mrs_uav_flightforge_simulator::srv::SetOrientation_Request>()
{
  return mrs_uav_flightforge_simulator::srv::builder::Init_SetOrientation_Request_quaternion();
}

}  // namespace mrs_uav_flightforge_simulator


namespace mrs_uav_flightforge_simulator
{

namespace srv
{

namespace builder
{

class Init_SetOrientation_Response_message
{
public:
  explicit Init_SetOrientation_Response_message(::mrs_uav_flightforge_simulator::srv::SetOrientation_Response & msg)
  : msg_(msg)
  {}
  ::mrs_uav_flightforge_simulator::srv::SetOrientation_Response message(::mrs_uav_flightforge_simulator::srv::SetOrientation_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mrs_uav_flightforge_simulator::srv::SetOrientation_Response msg_;
};

class Init_SetOrientation_Response_success
{
public:
  Init_SetOrientation_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetOrientation_Response_message success(::mrs_uav_flightforge_simulator::srv::SetOrientation_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetOrientation_Response_message(msg_);
  }

private:
  ::mrs_uav_flightforge_simulator::srv::SetOrientation_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mrs_uav_flightforge_simulator::srv::SetOrientation_Response>()
{
  return mrs_uav_flightforge_simulator::srv::builder::Init_SetOrientation_Response_success();
}

}  // namespace mrs_uav_flightforge_simulator


namespace mrs_uav_flightforge_simulator
{

namespace srv
{

namespace builder
{

class Init_SetOrientation_Event_response
{
public:
  explicit Init_SetOrientation_Event_response(::mrs_uav_flightforge_simulator::srv::SetOrientation_Event & msg)
  : msg_(msg)
  {}
  ::mrs_uav_flightforge_simulator::srv::SetOrientation_Event response(::mrs_uav_flightforge_simulator::srv::SetOrientation_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mrs_uav_flightforge_simulator::srv::SetOrientation_Event msg_;
};

class Init_SetOrientation_Event_request
{
public:
  explicit Init_SetOrientation_Event_request(::mrs_uav_flightforge_simulator::srv::SetOrientation_Event & msg)
  : msg_(msg)
  {}
  Init_SetOrientation_Event_response request(::mrs_uav_flightforge_simulator::srv::SetOrientation_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetOrientation_Event_response(msg_);
  }

private:
  ::mrs_uav_flightforge_simulator::srv::SetOrientation_Event msg_;
};

class Init_SetOrientation_Event_info
{
public:
  Init_SetOrientation_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetOrientation_Event_request info(::mrs_uav_flightforge_simulator::srv::SetOrientation_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetOrientation_Event_request(msg_);
  }

private:
  ::mrs_uav_flightforge_simulator::srv::SetOrientation_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mrs_uav_flightforge_simulator::srv::SetOrientation_Event>()
{
  return mrs_uav_flightforge_simulator::srv::builder::Init_SetOrientation_Event_info();
}

}  // namespace mrs_uav_flightforge_simulator

#endif  // MRS_UAV_FLIGHTFORGE_SIMULATOR__SRV__DETAIL__SET_ORIENTATION__BUILDER_HPP_
