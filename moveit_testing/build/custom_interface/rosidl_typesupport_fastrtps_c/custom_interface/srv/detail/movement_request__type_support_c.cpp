// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from custom_interface:srv/MovementRequest.idl
// generated code does not contain a copyright notice
#include "custom_interface/srv/detail/movement_request__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "custom_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "custom_interface/srv/detail/movement_request__struct.h"
#include "custom_interface/srv/detail/movement_request__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // positions
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // positions
#include "rosidl_runtime_c/string.h"  // command, constraints_identifier
#include "rosidl_runtime_c/string_functions.h"  // command, constraints_identifier

// forward declare type support functions


using _MovementRequest_Request__ros_msg_type = custom_interface__srv__MovementRequest_Request;

static bool _MovementRequest_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MovementRequest_Request__ros_msg_type * ros_message = static_cast<const _MovementRequest_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: command
  {
    const rosidl_runtime_c__String * str = &ros_message->command;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: positions
  {
    size_t size = ros_message->positions.size;
    auto array_ptr = ros_message->positions.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: constraints_identifier
  {
    const rosidl_runtime_c__String * str = &ros_message->constraints_identifier;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _MovementRequest_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MovementRequest_Request__ros_msg_type * ros_message = static_cast<_MovementRequest_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: command
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->command.data) {
      rosidl_runtime_c__String__init(&ros_message->command);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->command,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'command'\n");
      return false;
    }
  }

  // Field name: positions
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->positions.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->positions);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->positions, size)) {
      fprintf(stderr, "failed to create array for field 'positions'");
      return false;
    }
    auto array_ptr = ros_message->positions.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: constraints_identifier
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->constraints_identifier.data) {
      rosidl_runtime_c__String__init(&ros_message->constraints_identifier);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->constraints_identifier,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'constraints_identifier'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_interface
size_t get_serialized_size_custom_interface__srv__MovementRequest_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MovementRequest_Request__ros_msg_type * ros_message = static_cast<const _MovementRequest_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name command
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->command.size + 1);
  // field.name positions
  {
    size_t array_size = ros_message->positions.size;
    auto array_ptr = ros_message->positions.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name constraints_identifier
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->constraints_identifier.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _MovementRequest_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_custom_interface__srv__MovementRequest_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_interface
size_t max_serialized_size_custom_interface__srv__MovementRequest_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: command
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: positions
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: constraints_identifier
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = custom_interface__srv__MovementRequest_Request;
    is_plain =
      (
      offsetof(DataType, constraints_identifier) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MovementRequest_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_custom_interface__srv__MovementRequest_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MovementRequest_Request = {
  "custom_interface::srv",
  "MovementRequest_Request",
  _MovementRequest_Request__cdr_serialize,
  _MovementRequest_Request__cdr_deserialize,
  _MovementRequest_Request__get_serialized_size,
  _MovementRequest_Request__max_serialized_size
};

static rosidl_message_type_support_t _MovementRequest_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MovementRequest_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, srv, MovementRequest_Request)() {
  return &_MovementRequest_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "custom_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "custom_interface/srv/detail/movement_request__struct.h"
// already included above
// #include "custom_interface/srv/detail/movement_request__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _MovementRequest_Response__ros_msg_type = custom_interface__srv__MovementRequest_Response;

static bool _MovementRequest_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MovementRequest_Response__ros_msg_type * ros_message = static_cast<const _MovementRequest_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

static bool _MovementRequest_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MovementRequest_Response__ros_msg_type * ros_message = static_cast<_MovementRequest_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_interface
size_t get_serialized_size_custom_interface__srv__MovementRequest_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MovementRequest_Response__ros_msg_type * ros_message = static_cast<const _MovementRequest_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MovementRequest_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_custom_interface__srv__MovementRequest_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_custom_interface
size_t max_serialized_size_custom_interface__srv__MovementRequest_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = custom_interface__srv__MovementRequest_Response;
    is_plain =
      (
      offsetof(DataType, success) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MovementRequest_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_custom_interface__srv__MovementRequest_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MovementRequest_Response = {
  "custom_interface::srv",
  "MovementRequest_Response",
  _MovementRequest_Response__cdr_serialize,
  _MovementRequest_Response__cdr_deserialize,
  _MovementRequest_Response__get_serialized_size,
  _MovementRequest_Response__max_serialized_size
};

static rosidl_message_type_support_t _MovementRequest_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MovementRequest_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, srv, MovementRequest_Response)() {
  return &_MovementRequest_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "custom_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "custom_interface/srv/movement_request.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t MovementRequest__callbacks = {
  "custom_interface::srv",
  "MovementRequest",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, srv, MovementRequest_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, srv, MovementRequest_Response)(),
};

static rosidl_service_type_support_t MovementRequest__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &MovementRequest__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, srv, MovementRequest)() {
  return &MovementRequest__handle;
}

#if defined(__cplusplus)
}
#endif
