// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_interface:msg/TaskMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_interface/msg/detail/task_msg__rosidl_typesupport_introspection_c.h"
#include "custom_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_interface/msg/detail/task_msg__functions.h"
#include "custom_interface/msg/detail/task_msg__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interface__msg__TaskMsg__init(message_memory);
}

void custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_fini_function(void * message_memory)
{
  custom_interface__msg__TaskMsg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_message_member_array[3] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface__msg__TaskMsg, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "age",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface__msg__TaskMsg, age),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_student",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface__msg__TaskMsg, is_student),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_message_members = {
  "custom_interface__msg",  // message namespace
  "TaskMsg",  // message name
  3,  // number of fields
  sizeof(custom_interface__msg__TaskMsg),
  custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_message_member_array,  // message members
  custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_message_type_support_handle = {
  0,
  &custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, msg, TaskMsg)() {
  if (!custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_message_type_support_handle.typesupport_identifier) {
    custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interface__msg__TaskMsg__rosidl_typesupport_introspection_c__TaskMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
