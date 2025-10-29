// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_interface:srv/MovementRequest.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_interface/srv/detail/movement_request__rosidl_typesupport_introspection_c.h"
#include "custom_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_interface/srv/detail/movement_request__functions.h"
#include "custom_interface/srv/detail/movement_request__struct.h"


// Include directives for member types
// Member `command`
// Member `constraints_identifier`
#include "rosidl_runtime_c/string_functions.h"
// Member `positions`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interface__srv__MovementRequest_Request__init(message_memory);
}

void custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_fini_function(void * message_memory)
{
  custom_interface__srv__MovementRequest_Request__fini(message_memory);
}

size_t custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__size_function__MovementRequest_Request__positions(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__get_const_function__MovementRequest_Request__positions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__get_function__MovementRequest_Request__positions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__fetch_function__MovementRequest_Request__positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__get_const_function__MovementRequest_Request__positions(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__assign_function__MovementRequest_Request__positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__get_function__MovementRequest_Request__positions(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__resize_function__MovementRequest_Request__positions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_message_member_array[3] = {
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface__srv__MovementRequest_Request, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface__srv__MovementRequest_Request, positions),  // bytes offset in struct
    NULL,  // default value
    custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__size_function__MovementRequest_Request__positions,  // size() function pointer
    custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__get_const_function__MovementRequest_Request__positions,  // get_const(index) function pointer
    custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__get_function__MovementRequest_Request__positions,  // get(index) function pointer
    custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__fetch_function__MovementRequest_Request__positions,  // fetch(index, &value) function pointer
    custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__assign_function__MovementRequest_Request__positions,  // assign(index, value) function pointer
    custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__resize_function__MovementRequest_Request__positions  // resize(index) function pointer
  },
  {
    "constraints_identifier",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface__srv__MovementRequest_Request, constraints_identifier),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_message_members = {
  "custom_interface__srv",  // message namespace
  "MovementRequest_Request",  // message name
  3,  // number of fields
  sizeof(custom_interface__srv__MovementRequest_Request),
  custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_message_member_array,  // message members
  custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_message_type_support_handle = {
  0,
  &custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, srv, MovementRequest_Request)() {
  if (!custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_message_type_support_handle.typesupport_identifier) {
    custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interface__srv__MovementRequest_Request__rosidl_typesupport_introspection_c__MovementRequest_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interface/srv/detail/movement_request__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interface/srv/detail/movement_request__functions.h"
// already included above
// #include "custom_interface/srv/detail/movement_request__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interface__srv__MovementRequest_Response__init(message_memory);
}

void custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_fini_function(void * message_memory)
{
  custom_interface__srv__MovementRequest_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface__srv__MovementRequest_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_message_members = {
  "custom_interface__srv",  // message namespace
  "MovementRequest_Response",  // message name
  1,  // number of fields
  sizeof(custom_interface__srv__MovementRequest_Response),
  custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_message_member_array,  // message members
  custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_message_type_support_handle = {
  0,
  &custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, srv, MovementRequest_Response)() {
  if (!custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_message_type_support_handle.typesupport_identifier) {
    custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interface__srv__MovementRequest_Response__rosidl_typesupport_introspection_c__MovementRequest_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "custom_interface/srv/detail/movement_request__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers custom_interface__srv__detail__movement_request__rosidl_typesupport_introspection_c__MovementRequest_service_members = {
  "custom_interface__srv",  // service namespace
  "MovementRequest",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // custom_interface__srv__detail__movement_request__rosidl_typesupport_introspection_c__MovementRequest_Request_message_type_support_handle,
  NULL  // response message
  // custom_interface__srv__detail__movement_request__rosidl_typesupport_introspection_c__MovementRequest_Response_message_type_support_handle
};

static rosidl_service_type_support_t custom_interface__srv__detail__movement_request__rosidl_typesupport_introspection_c__MovementRequest_service_type_support_handle = {
  0,
  &custom_interface__srv__detail__movement_request__rosidl_typesupport_introspection_c__MovementRequest_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, srv, MovementRequest_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, srv, MovementRequest_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, srv, MovementRequest)() {
  if (!custom_interface__srv__detail__movement_request__rosidl_typesupport_introspection_c__MovementRequest_service_type_support_handle.typesupport_identifier) {
    custom_interface__srv__detail__movement_request__rosidl_typesupport_introspection_c__MovementRequest_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)custom_interface__srv__detail__movement_request__rosidl_typesupport_introspection_c__MovementRequest_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, srv, MovementRequest_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, srv, MovementRequest_Response)()->data;
  }

  return &custom_interface__srv__detail__movement_request__rosidl_typesupport_introspection_c__MovementRequest_service_type_support_handle;
}
