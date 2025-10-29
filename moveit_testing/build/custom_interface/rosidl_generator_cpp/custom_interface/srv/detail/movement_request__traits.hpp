// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface:srv/MovementRequest.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__TRAITS_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface/srv/detail/movement_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const MovementRequest_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: positions
  {
    if (msg.positions.size() == 0) {
      out << "positions: []";
    } else {
      out << "positions: [";
      size_t pending_items = msg.positions.size();
      for (auto item : msg.positions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: constraints_identifier
  {
    out << "constraints_identifier: ";
    rosidl_generator_traits::value_to_yaml(msg.constraints_identifier, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MovementRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.positions.size() == 0) {
      out << "positions: []\n";
    } else {
      out << "positions:\n";
      for (auto item : msg.positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: constraints_identifier
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "constraints_identifier: ";
    rosidl_generator_traits::value_to_yaml(msg.constraints_identifier, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MovementRequest_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interface

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface::srv::MovementRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::srv::MovementRequest_Request & msg)
{
  return custom_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::srv::MovementRequest_Request>()
{
  return "custom_interface::srv::MovementRequest_Request";
}

template<>
inline const char * name<custom_interface::srv::MovementRequest_Request>()
{
  return "custom_interface/srv/MovementRequest_Request";
}

template<>
struct has_fixed_size<custom_interface::srv::MovementRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interface::srv::MovementRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interface::srv::MovementRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const MovementRequest_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MovementRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MovementRequest_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interface

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface::srv::MovementRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::srv::MovementRequest_Response & msg)
{
  return custom_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::srv::MovementRequest_Response>()
{
  return "custom_interface::srv::MovementRequest_Response";
}

template<>
inline const char * name<custom_interface::srv::MovementRequest_Response>()
{
  return "custom_interface/srv/MovementRequest_Response";
}

template<>
struct has_fixed_size<custom_interface::srv::MovementRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interface::srv::MovementRequest_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interface::srv::MovementRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interface::srv::MovementRequest>()
{
  return "custom_interface::srv::MovementRequest";
}

template<>
inline const char * name<custom_interface::srv::MovementRequest>()
{
  return "custom_interface/srv/MovementRequest";
}

template<>
struct has_fixed_size<custom_interface::srv::MovementRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interface::srv::MovementRequest_Request>::value &&
    has_fixed_size<custom_interface::srv::MovementRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interface::srv::MovementRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interface::srv::MovementRequest_Request>::value &&
    has_bounded_size<custom_interface::srv::MovementRequest_Response>::value
  >
{
};

template<>
struct is_service<custom_interface::srv::MovementRequest>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interface::srv::MovementRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interface::srv::MovementRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__TRAITS_HPP_
