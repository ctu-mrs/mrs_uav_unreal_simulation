// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mrs_uav_flightforge_simulator:srv/SetOrientation.idl
// generated code does not contain a copyright notice

#include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mrs_uav_flightforge_simulator
const rosidl_type_hash_t *
mrs_uav_flightforge_simulator__srv__SetOrientation__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x38, 0x1b, 0xe4, 0x2b, 0x66, 0xce, 0x51, 0xe5,
      0xe6, 0x8f, 0xca, 0x51, 0x92, 0x10, 0x5b, 0xd4,
      0x09, 0xd8, 0x9b, 0xf8, 0x07, 0x38, 0x03, 0x25,
      0xfc, 0x16, 0x84, 0x28, 0xf5, 0x2b, 0x13, 0xa6,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mrs_uav_flightforge_simulator
const rosidl_type_hash_t *
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x32, 0xaa, 0x52, 0xda, 0x78, 0x1e, 0x32, 0x8b,
      0x71, 0x1b, 0xa8, 0xd6, 0x03, 0x3a, 0xd5, 0xf3,
      0xf8, 0x67, 0x08, 0x89, 0xd4, 0x67, 0xd9, 0x30,
      0x3a, 0xee, 0x6b, 0x55, 0x8b, 0x4c, 0x57, 0x83,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mrs_uav_flightforge_simulator
const rosidl_type_hash_t *
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xee, 0x6b, 0xf6, 0xa3, 0x13, 0x4c, 0xbd, 0x84,
      0x88, 0xe8, 0xb0, 0x3a, 0x1d, 0x0c, 0x30, 0xd2,
      0x52, 0xb7, 0xdb, 0xda, 0x72, 0x98, 0xfa, 0xdc,
      0xbf, 0xc8, 0xcc, 0x55, 0xe6, 0x40, 0x15, 0xb8,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mrs_uav_flightforge_simulator
const rosidl_type_hash_t *
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0b, 0x4f, 0xdd, 0x8e, 0x97, 0x5a, 0xee, 0xf4,
      0xa7, 0xe7, 0x3b, 0x48, 0x67, 0x8d, 0xb1, 0xea,
      0xd1, 0x79, 0x8e, 0x16, 0xe6, 0x83, 0xe8, 0x62,
      0x99, 0x41, 0x66, 0xd8, 0xea, 0x17, 0x88, 0x86,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/quaternion_stamped__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__QuaternionStamped__EXPECTED_HASH = {1, {
    0x38, 0x1a, 0xdd, 0x86, 0xc6, 0xc3, 0x16, 0x06,
    0x44, 0xd2, 0x28, 0xca, 0x34, 0x21, 0x82, 0xc7,
    0xfd, 0x6c, 0x7f, 0xab, 0x11, 0xc7, 0xa8, 0x5a,
    0xd8, 0x17, 0xa9, 0xcc, 0x22, 0xdb, 0xac, 0x6e,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char mrs_uav_flightforge_simulator__srv__SetOrientation__TYPE_NAME[] = "mrs_uav_flightforge_simulator/srv/SetOrientation";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char geometry_msgs__msg__QuaternionStamped__TYPE_NAME[] = "geometry_msgs/msg/QuaternionStamped";
static char mrs_uav_flightforge_simulator__srv__SetOrientation_Event__TYPE_NAME[] = "mrs_uav_flightforge_simulator/srv/SetOrientation_Event";
static char mrs_uav_flightforge_simulator__srv__SetOrientation_Request__TYPE_NAME[] = "mrs_uav_flightforge_simulator/srv/SetOrientation_Request";
static char mrs_uav_flightforge_simulator__srv__SetOrientation_Response__TYPE_NAME[] = "mrs_uav_flightforge_simulator/srv/SetOrientation_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mrs_uav_flightforge_simulator__srv__SetOrientation__FIELD_NAME__request_message[] = "request_message";
static char mrs_uav_flightforge_simulator__srv__SetOrientation__FIELD_NAME__response_message[] = "response_message";
static char mrs_uav_flightforge_simulator__srv__SetOrientation__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mrs_uav_flightforge_simulator__srv__SetOrientation__FIELDS[] = {
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Request__TYPE_NAME, 56, 56},
    },
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Response__TYPE_NAME, 57, 57},
    },
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Event__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mrs_uav_flightforge_simulator__srv__SetOrientation__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__QuaternionStamped__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Event__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Request__TYPE_NAME, 56, 56},
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Response__TYPE_NAME, 57, 57},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mrs_uav_flightforge_simulator__srv__SetOrientation__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mrs_uav_flightforge_simulator__srv__SetOrientation__TYPE_NAME, 48, 48},
      {mrs_uav_flightforge_simulator__srv__SetOrientation__FIELDS, 3, 3},
    },
    {mrs_uav_flightforge_simulator__srv__SetOrientation__REFERENCED_TYPE_DESCRIPTIONS, 8, 8},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__QuaternionStamped__EXPECTED_HASH, geometry_msgs__msg__QuaternionStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__QuaternionStamped__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mrs_uav_flightforge_simulator__srv__SetOrientation_Request__FIELD_NAME__quaternion[] = "quaternion";

static rosidl_runtime_c__type_description__Field mrs_uav_flightforge_simulator__srv__SetOrientation_Request__FIELDS[] = {
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Request__FIELD_NAME__quaternion, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__QuaternionStamped__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mrs_uav_flightforge_simulator__srv__SetOrientation_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__QuaternionStamped__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Request__TYPE_NAME, 56, 56},
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Request__FIELDS, 1, 1},
    },
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Request__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__QuaternionStamped__EXPECTED_HASH, geometry_msgs__msg__QuaternionStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__QuaternionStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mrs_uav_flightforge_simulator__srv__SetOrientation_Response__FIELD_NAME__success[] = "success";
static char mrs_uav_flightforge_simulator__srv__SetOrientation_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field mrs_uav_flightforge_simulator__srv__SetOrientation_Response__FIELDS[] = {
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Response__TYPE_NAME, 57, 57},
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mrs_uav_flightforge_simulator__srv__SetOrientation_Event__FIELD_NAME__info[] = "info";
static char mrs_uav_flightforge_simulator__srv__SetOrientation_Event__FIELD_NAME__request[] = "request";
static char mrs_uav_flightforge_simulator__srv__SetOrientation_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mrs_uav_flightforge_simulator__srv__SetOrientation_Event__FIELDS[] = {
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Request__TYPE_NAME, 56, 56},
    },
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Response__TYPE_NAME, 57, 57},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mrs_uav_flightforge_simulator__srv__SetOrientation_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__QuaternionStamped__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Request__TYPE_NAME, 56, 56},
    {NULL, 0, 0},
  },
  {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Response__TYPE_NAME, 57, 57},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Event__TYPE_NAME, 54, 54},
      {mrs_uav_flightforge_simulator__srv__SetOrientation_Event__FIELDS, 3, 3},
    },
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Event__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__QuaternionStamped__EXPECTED_HASH, geometry_msgs__msg__QuaternionStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__QuaternionStamped__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "geometry_msgs/QuaternionStamped quaternion\n"
  "---\n"
  "bool success\n"
  "string message";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mrs_uav_flightforge_simulator__srv__SetOrientation__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mrs_uav_flightforge_simulator__srv__SetOrientation__TYPE_NAME, 48, 48},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 75, 75},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Request__TYPE_NAME, 56, 56},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Response__TYPE_NAME, 57, 57},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mrs_uav_flightforge_simulator__srv__SetOrientation_Event__TYPE_NAME, 54, 54},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mrs_uav_flightforge_simulator__srv__SetOrientation__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[9];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 9, 9};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mrs_uav_flightforge_simulator__srv__SetOrientation__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__QuaternionStamped__get_individual_type_description_source(NULL);
    sources[4] = *mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_individual_type_description_source(NULL);
    sources[5] = *mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_individual_type_description_source(NULL);
    sources[6] = *mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[8] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__QuaternionStamped__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mrs_uav_flightforge_simulator__srv__SetOrientation_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__QuaternionStamped__get_individual_type_description_source(NULL);
    sources[4] = *mrs_uav_flightforge_simulator__srv__SetOrientation_Request__get_individual_type_description_source(NULL);
    sources[5] = *mrs_uav_flightforge_simulator__srv__SetOrientation_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[7] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
