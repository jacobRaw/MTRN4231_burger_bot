// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:srv/MovementRequest.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__STRUCT_H_
#define CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
// Member 'constraints_identifier'
#include "rosidl_runtime_c/string.h"
// Member 'positions'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/MovementRequest in the package custom_interface.
typedef struct custom_interface__srv__MovementRequest_Request
{
  rosidl_runtime_c__String command;
  rosidl_runtime_c__double__Sequence positions;
  rosidl_runtime_c__String constraints_identifier;
} custom_interface__srv__MovementRequest_Request;

// Struct for a sequence of custom_interface__srv__MovementRequest_Request.
typedef struct custom_interface__srv__MovementRequest_Request__Sequence
{
  custom_interface__srv__MovementRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__MovementRequest_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MovementRequest in the package custom_interface.
typedef struct custom_interface__srv__MovementRequest_Response
{
  bool success;
} custom_interface__srv__MovementRequest_Response;

// Struct for a sequence of custom_interface__srv__MovementRequest_Response.
typedef struct custom_interface__srv__MovementRequest_Response__Sequence
{
  custom_interface__srv__MovementRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__MovementRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__MOVEMENT_REQUEST__STRUCT_H_
