// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mrs_uav_flightforge_simulator:srv/SetOrientation.idl
// generated code does not contain a copyright notice
#include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `quaternion`
#include "geometry_msgs/msg/detail/quaternion_stamped__functions.h"

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__init(mrs_uav_flightforge_simulator__srv__SetOrientation_Request * msg)
{
  if (!msg) {
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__QuaternionStamped__init(&msg->quaternion)) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Request__fini(msg);
    return false;
  }
  return true;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__fini(mrs_uav_flightforge_simulator__srv__SetOrientation_Request * msg)
{
  if (!msg) {
    return;
  }
  // quaternion
  geometry_msgs__msg__QuaternionStamped__fini(&msg->quaternion);
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__are_equal(const mrs_uav_flightforge_simulator__srv__SetOrientation_Request * lhs, const mrs_uav_flightforge_simulator__srv__SetOrientation_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__QuaternionStamped__are_equal(
      &(lhs->quaternion), &(rhs->quaternion)))
  {
    return false;
  }
  return true;
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__copy(
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Request * input,
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__QuaternionStamped__copy(
      &(input->quaternion), &(output->quaternion)))
  {
    return false;
  }
  return true;
}

mrs_uav_flightforge_simulator__srv__SetOrientation_Request *
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request * msg = (mrs_uav_flightforge_simulator__srv__SetOrientation_Request *)allocator.allocate(sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Request));
  bool success = mrs_uav_flightforge_simulator__srv__SetOrientation_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__destroy(mrs_uav_flightforge_simulator__srv__SetOrientation_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__init(mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request * data = NULL;

  if (size) {
    data = (mrs_uav_flightforge_simulator__srv__SetOrientation_Request *)allocator.zero_allocate(size, sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mrs_uav_flightforge_simulator__srv__SetOrientation_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mrs_uav_flightforge_simulator__srv__SetOrientation_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__fini(mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mrs_uav_flightforge_simulator__srv__SetOrientation_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence *
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * array = (mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence *)allocator.allocate(sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__destroy(mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__are_equal(const mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * lhs, const mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__copy(
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * input,
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mrs_uav_flightforge_simulator__srv__SetOrientation_Request * data =
      (mrs_uav_flightforge_simulator__srv__SetOrientation_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mrs_uav_flightforge_simulator__srv__SetOrientation_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__init(mrs_uav_flightforge_simulator__srv__SetOrientation_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Response__fini(msg);
    return false;
  }
  return true;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__fini(mrs_uav_flightforge_simulator__srv__SetOrientation_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__are_equal(const mrs_uav_flightforge_simulator__srv__SetOrientation_Response * lhs, const mrs_uav_flightforge_simulator__srv__SetOrientation_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__copy(
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Response * input,
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

mrs_uav_flightforge_simulator__srv__SetOrientation_Response *
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response * msg = (mrs_uav_flightforge_simulator__srv__SetOrientation_Response *)allocator.allocate(sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Response));
  bool success = mrs_uav_flightforge_simulator__srv__SetOrientation_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__destroy(mrs_uav_flightforge_simulator__srv__SetOrientation_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__init(mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response * data = NULL;

  if (size) {
    data = (mrs_uav_flightforge_simulator__srv__SetOrientation_Response *)allocator.zero_allocate(size, sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mrs_uav_flightforge_simulator__srv__SetOrientation_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mrs_uav_flightforge_simulator__srv__SetOrientation_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__fini(mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mrs_uav_flightforge_simulator__srv__SetOrientation_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence *
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * array = (mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence *)allocator.allocate(sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__destroy(mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__are_equal(const mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * lhs, const mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__copy(
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * input,
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mrs_uav_flightforge_simulator__srv__SetOrientation_Response * data =
      (mrs_uav_flightforge_simulator__srv__SetOrientation_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mrs_uav_flightforge_simulator__srv__SetOrientation_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "mrs_uav_flightforge_simulator/srv/detail/set_orientation__functions.h"

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__init(mrs_uav_flightforge_simulator__srv__SetOrientation_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__fini(msg);
    return false;
  }
  // request
  if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__init(&msg->request, 0)) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__fini(msg);
    return false;
  }
  // response
  if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__init(&msg->response, 0)) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__fini(msg);
    return false;
  }
  return true;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__fini(mrs_uav_flightforge_simulator__srv__SetOrientation_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__fini(&msg->request);
  // response
  mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__fini(&msg->response);
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__are_equal(const mrs_uav_flightforge_simulator__srv__SetOrientation_Event * lhs, const mrs_uav_flightforge_simulator__srv__SetOrientation_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__copy(
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Event * input,
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

mrs_uav_flightforge_simulator__srv__SetOrientation_Event *
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event * msg = (mrs_uav_flightforge_simulator__srv__SetOrientation_Event *)allocator.allocate(sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Event));
  bool success = mrs_uav_flightforge_simulator__srv__SetOrientation_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__destroy(mrs_uav_flightforge_simulator__srv__SetOrientation_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence__init(mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event * data = NULL;

  if (size) {
    data = (mrs_uav_flightforge_simulator__srv__SetOrientation_Event *)allocator.zero_allocate(size, sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mrs_uav_flightforge_simulator__srv__SetOrientation_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mrs_uav_flightforge_simulator__srv__SetOrientation_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence__fini(mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mrs_uav_flightforge_simulator__srv__SetOrientation_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence *
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence * array = (mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence *)allocator.allocate(sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence__destroy(mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence__are_equal(const mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence * lhs, const mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence__copy(
  const mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence * input,
  mrs_uav_flightforge_simulator__srv__SetOrientation_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mrs_uav_flightforge_simulator__srv__SetOrientation_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mrs_uav_flightforge_simulator__srv__SetOrientation_Event * data =
      (mrs_uav_flightforge_simulator__srv__SetOrientation_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mrs_uav_flightforge_simulator__srv__SetOrientation_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mrs_uav_flightforge_simulator__srv__SetOrientation_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
