// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mrs_uav_flightforge_simulator:srv/SetOrientation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mrs_uav_flightforge_simulator/srv/set_orientation.h"


#ifndef MRS_UAV_FLIGHTFORGE_SIMULATOR__SRV__DETAIL__SET_ORIENTATION__STRUCT_H_
#define MRS_UAV_FLIGHTFORGE_SIMULATOR__SRV__DETAIL__SET_ORIENTATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion_stamped__struct.h"

/// Struct defined in srv/SetOrientation in the package mrs_uav_flightforge_simulator.
typedef struct mrs_uav_flightforge_simulator__srv__SetOrientation_Request
{
  geometry_msgs__msg__QuaternionStamped quaternion;
} mrs_uav_flightforge_simulator__srv__SetOrientation_Request;

// Struct for a sequence of mrs_uav_flightforge_simulator__srv__SetOrientation_Request.
typedef struct mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetOrientation in the package mrs_uav_flightforge_simulator.
typedef struct mrs_uav_flightforge_simulator__srv__SetOrientation_Response
{
  bool success;
  rosidl_runtime_c__String message;
} mrs_uav_flightforge_simulator__srv__SetOrientation_Response;

// Struct for a sequence of mrs_uav_flightforge_simulator__srv__SetOrientation_Response.
typedef struct mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetOrientation in the package mrs_uav_flightforge_simulator.
typedef struct mrs_uav_flightforge_simulator__srv__SetOrientation_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence request;
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence response;
} mrs_uav_flightforge_simulator__srv__SetOrientation_Event;

// Struct for a sequence of mrs_uav_flightforge_simulator__srv__SetOrientation_Event.
typedef struct mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence
{
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MRS_UAV_FLIGHTFORGE_SIMULATOR__SRV__DETAIL__SET_ORIENTATION__STRUCT_H_
