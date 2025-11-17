// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interface:srv/MovementRequest.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__STRUCT_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interface__srv__MovementRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface__srv__MovementRequest_Request __declspec(deprecated)
#endif

namespace custom_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MovementRequest_Request_
{
  using Type = MovementRequest_Request_<ContainerAllocator>;

  explicit MovementRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->constraints_identifier = "";
    }
  }

  explicit MovementRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc),
    constraints_identifier(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->constraints_identifier = "";
    }
  }

  // field types and members
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;
  using _positions_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _positions_type positions;
  using _constraints_identifier_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _constraints_identifier_type constraints_identifier;

  // setters for named parameter idiom
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__positions(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->positions = _arg;
    return *this;
  }
  Type & set__constraints_identifier(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->constraints_identifier = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface::srv::MovementRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface::srv::MovementRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface::srv::MovementRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface::srv::MovementRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface::srv::MovementRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface::srv::MovementRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface::srv::MovementRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface::srv::MovementRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface::srv::MovementRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface::srv::MovementRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface__srv__MovementRequest_Request
    std::shared_ptr<custom_interface::srv::MovementRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface__srv__MovementRequest_Request
    std::shared_ptr<custom_interface::srv::MovementRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MovementRequest_Request_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->positions != other.positions) {
      return false;
    }
    if (this->constraints_identifier != other.constraints_identifier) {
      return false;
    }
    return true;
  }
  bool operator!=(const MovementRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MovementRequest_Request_

// alias to use template instance with default allocator
using MovementRequest_Request =
  custom_interface::srv::MovementRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interface


#ifndef _WIN32
# define DEPRECATED__custom_interface__srv__MovementRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface__srv__MovementRequest_Response __declspec(deprecated)
#endif

namespace custom_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MovementRequest_Response_
{
  using Type = MovementRequest_Response_<ContainerAllocator>;

  explicit MovementRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit MovementRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface::srv::MovementRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface::srv::MovementRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface::srv::MovementRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface::srv::MovementRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface::srv::MovementRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface::srv::MovementRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface::srv::MovementRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface::srv::MovementRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface::srv::MovementRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface::srv::MovementRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface__srv__MovementRequest_Response
    std::shared_ptr<custom_interface::srv::MovementRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface__srv__MovementRequest_Response
    std::shared_ptr<custom_interface::srv::MovementRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MovementRequest_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const MovementRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MovementRequest_Response_

// alias to use template instance with default allocator
using MovementRequest_Response =
  custom_interface::srv::MovementRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interface

namespace custom_interface
{

namespace srv
{

struct MovementRequest
{
  using Request = custom_interface::srv::MovementRequest_Request;
  using Response = custom_interface::srv::MovementRequest_Response;
};

}  // namespace srv

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__STRUCT_HPP_
