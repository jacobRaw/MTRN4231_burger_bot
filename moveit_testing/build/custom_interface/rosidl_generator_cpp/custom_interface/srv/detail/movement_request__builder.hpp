// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:srv/MovementRequest.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__BUILDER_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/srv/detail/movement_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_MovementRequest_Request_constraints_identifier
{
public:
  explicit Init_MovementRequest_Request_constraints_identifier(::custom_interface::srv::MovementRequest_Request & msg)
  : msg_(msg)
  {}
  ::custom_interface::srv::MovementRequest_Request constraints_identifier(::custom_interface::srv::MovementRequest_Request::_constraints_identifier_type arg)
  {
    msg_.constraints_identifier = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::MovementRequest_Request msg_;
};

class Init_MovementRequest_Request_positions
{
public:
  explicit Init_MovementRequest_Request_positions(::custom_interface::srv::MovementRequest_Request & msg)
  : msg_(msg)
  {}
  Init_MovementRequest_Request_constraints_identifier positions(::custom_interface::srv::MovementRequest_Request::_positions_type arg)
  {
    msg_.positions = std::move(arg);
    return Init_MovementRequest_Request_constraints_identifier(msg_);
  }

private:
  ::custom_interface::srv::MovementRequest_Request msg_;
};

class Init_MovementRequest_Request_command
{
public:
  Init_MovementRequest_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovementRequest_Request_positions command(::custom_interface::srv::MovementRequest_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_MovementRequest_Request_positions(msg_);
  }

private:
  ::custom_interface::srv::MovementRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::MovementRequest_Request>()
{
  return custom_interface::srv::builder::Init_MovementRequest_Request_command();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_MovementRequest_Response_success
{
public:
  Init_MovementRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface::srv::MovementRequest_Response success(::custom_interface::srv::MovementRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::MovementRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::MovementRequest_Response>()
{
  return custom_interface::srv::builder::Init_MovementRequest_Response_success();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__BUILDER_HPP_
