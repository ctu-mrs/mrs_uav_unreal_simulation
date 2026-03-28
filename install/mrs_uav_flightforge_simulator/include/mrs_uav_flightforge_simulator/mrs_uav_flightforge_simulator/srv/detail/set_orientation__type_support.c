// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mrs_uav_flightforge_simulator:srv/SetOrientation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__rosidl_typesupport_introspection_c.h"
#include "mrs_uav_flightforge_simulator/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__functions.h"
#include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__struct.h"


// Include directives for member types
// Member `quaternion`
#include "geometry_msgs/msg/quaternion_stamped.h"
// Member `quaternion`
#include "geometry_msgs/msg/detail/quaternion_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__init(message_memory);
}

void mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_fini_function(void * message_memory)
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_member_array[1] = {
  {
    "quaternion",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mrs_uav_flightforge_simulator__srv__SetOrientation_Request, quaternion),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_members = {
  "mrs_uav_flightforge_simulator__srv",  // message namespace
  "SetOrientation_Request",  // message name
  1,  // number of fields
  sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Request),
  false,  // has_any_key_member_
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_member_array,  // message members
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_type_support_handle = {
  0,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_members,
  get_message_typesupport_handle_function,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_type_hash,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_type_description,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mrs_uav_flightforge_simulator
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Request)() {
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, QuaternionStamped)();
  if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_type_support_handle.typesupport_identifier) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mrs_uav_flightforge_simulator/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__functions.h"
// already included above
// #include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__init(message_memory);
}

void mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_fini_function(void * message_memory)
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mrs_uav_flightforge_simulator__srv__SetOrientation_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mrs_uav_flightforge_simulator__srv__SetOrientation_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_message_members = {
  "mrs_uav_flightforge_simulator__srv",  // message namespace
  "SetOrientation_Response",  // message name
  2,  // number of fields
  sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Response),
  false,  // has_any_key_member_
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_message_member_array,  // message members
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_message_type_support_handle = {
  0,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_message_members,
  get_message_typesupport_handle_function,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_type_hash,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_type_description,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mrs_uav_flightforge_simulator
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Response)() {
  if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_message_type_support_handle.typesupport_identifier) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mrs_uav_flightforge_simulator/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__functions.h"
// already included above
// #include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "mrs_uav_flightforge_simulator/srv/set_orientation.h"
// Member `request`
// Member `response`
// already included above
// #include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__init(message_memory);
}

void mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_fini_function(void * message_memory)
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__fini(message_memory);
}

size_t mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__size_function__SetOrientation_Event__request(
  const void * untyped_member)
{
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * member =
    (const mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_const_function__SetOrientation_Event__request(
  const void * untyped_member, size_t index)
{
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * member =
    (const mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_function__SetOrientation_Event__request(
  void * untyped_member, size_t index)
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * member =
    (mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__fetch_function__SetOrientation_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Request * item =
    ((const mrs_uav_flightforge_simulator__srv__SetOrientation_Request *)
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_const_function__SetOrientation_Event__request(untyped_member, index));
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request * value =
    (mrs_uav_flightforge_simulator__srv__SetOrientation_Request *)(untyped_value);
  *value = *item;
}

void mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__assign_function__SetOrientation_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request * item =
    ((mrs_uav_flightforge_simulator__srv__SetOrientation_Request *)
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_function__SetOrientation_Event__request(untyped_member, index));
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Request * value =
    (const mrs_uav_flightforge_simulator__srv__SetOrientation_Request *)(untyped_value);
  *item = *value;
}

bool mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__resize_function__SetOrientation_Event__request(
  void * untyped_member, size_t size)
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * member =
    (mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence *)(untyped_member);
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__fini(member);
  return mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__init(member, size);
}

size_t mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__size_function__SetOrientation_Event__response(
  const void * untyped_member)
{
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * member =
    (const mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_const_function__SetOrientation_Event__response(
  const void * untyped_member, size_t index)
{
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * member =
    (const mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_function__SetOrientation_Event__response(
  void * untyped_member, size_t index)
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * member =
    (mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__fetch_function__SetOrientation_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Response * item =
    ((const mrs_uav_flightforge_simulator__srv__SetOrientation_Response *)
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_const_function__SetOrientation_Event__response(untyped_member, index));
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response * value =
    (mrs_uav_flightforge_simulator__srv__SetOrientation_Response *)(untyped_value);
  *value = *item;
}

void mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__assign_function__SetOrientation_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response * item =
    ((mrs_uav_flightforge_simulator__srv__SetOrientation_Response *)
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_function__SetOrientation_Event__response(untyped_member, index));
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Response * value =
    (const mrs_uav_flightforge_simulator__srv__SetOrientation_Response *)(untyped_value);
  *item = *value;
}

bool mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__resize_function__SetOrientation_Event__response(
  void * untyped_member, size_t size)
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * member =
    (mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence *)(untyped_member);
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__fini(member);
  return mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mrs_uav_flightforge_simulator__srv__SetOrientation_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mrs_uav_flightforge_simulator__srv__SetOrientation_Event, request),  // bytes offset in struct
    NULL,  // default value
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__size_function__SetOrientation_Event__request,  // size() function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_const_function__SetOrientation_Event__request,  // get_const(index) function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_function__SetOrientation_Event__request,  // get(index) function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__fetch_function__SetOrientation_Event__request,  // fetch(index, &value) function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__assign_function__SetOrientation_Event__request,  // assign(index, value) function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__resize_function__SetOrientation_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mrs_uav_flightforge_simulator__srv__SetOrientation_Event, response),  // bytes offset in struct
    NULL,  // default value
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__size_function__SetOrientation_Event__response,  // size() function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_const_function__SetOrientation_Event__response,  // get_const(index) function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__get_function__SetOrientation_Event__response,  // get(index) function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__fetch_function__SetOrientation_Event__response,  // fetch(index, &value) function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__assign_function__SetOrientation_Event__response,  // assign(index, value) function pointer
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__resize_function__SetOrientation_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_members = {
  "mrs_uav_flightforge_simulator__srv",  // message namespace
  "SetOrientation_Event",  // message name
  3,  // number of fields
  sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Event),
  false,  // has_any_key_member_
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_member_array,  // message members
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_type_support_handle = {
  0,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_members,
  get_message_typesupport_handle_function,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_type_hash,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_type_description,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mrs_uav_flightforge_simulator
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Event)() {
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Request)();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Response)();
  if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_type_support_handle.typesupport_identifier) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mrs_uav_flightforge_simulator/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_service_members = {
  "mrs_uav_flightforge_simulator__srv",  // service namespace
  "SetOrientation",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_Request_message_type_support_handle,
  NULL,  // response message
  // mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_Response_message_type_support_handle
  NULL  // event_message
  // mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_Response_message_type_support_handle
};


static rosidl_service_type_support_t mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_service_type_support_handle = {
  0,
  &mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_service_members,
  get_service_typesupport_handle_function,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Request__rosidl_typesupport_introspection_c__SetOrientation_Request_message_type_support_handle,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Response__rosidl_typesupport_introspection_c__SetOrientation_Response_message_type_support_handle,
  &mrs_uav_flightforge_simulator__srv__SetOrientation_Event__rosidl_typesupport_introspection_c__SetOrientation_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mrs_uav_flightforge_simulator,
    srv,
    SetOrientation
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mrs_uav_flightforge_simulator,
    srv,
    SetOrientation
  ),
  &mrs_uav_flightforge_simulator__srv__SetOrientation__get_type_hash,
  &mrs_uav_flightforge_simulator__srv__SetOrientation__get_type_description,
  &mrs_uav_flightforge_simulator__srv__SetOrientation__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mrs_uav_flightforge_simulator
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation)(void) {
  if (!mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_service_type_support_handle.typesupport_identifier) {
    mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mrs_uav_flightforge_simulator, srv, SetOrientation_Event)()->data;
  }

  return &mrs_uav_flightforge_simulator__srv__detail__set_orientation__rosidl_typesupport_introspection_c__SetOrientation_service_type_support_handle;
}
