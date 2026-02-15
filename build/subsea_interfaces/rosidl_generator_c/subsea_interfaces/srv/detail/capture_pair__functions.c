// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from subsea_interfaces:srv/CapturePair.idl
// generated code does not contain a copyright notice
#include "subsea_interfaces/srv/detail/capture_pair__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `session_id`
// Member `output_dir`
#include "rosidl_runtime_c/string_functions.h"

bool
subsea_interfaces__srv__CapturePair_Request__init(subsea_interfaces__srv__CapturePair_Request * msg)
{
  if (!msg) {
    return false;
  }
  // session_id
  if (!rosidl_runtime_c__String__init(&msg->session_id)) {
    subsea_interfaces__srv__CapturePair_Request__fini(msg);
    return false;
  }
  // output_dir
  if (!rosidl_runtime_c__String__init(&msg->output_dir)) {
    subsea_interfaces__srv__CapturePair_Request__fini(msg);
    return false;
  }
  // jpeg_quality
  return true;
}

void
subsea_interfaces__srv__CapturePair_Request__fini(subsea_interfaces__srv__CapturePair_Request * msg)
{
  if (!msg) {
    return;
  }
  // session_id
  rosidl_runtime_c__String__fini(&msg->session_id);
  // output_dir
  rosidl_runtime_c__String__fini(&msg->output_dir);
  // jpeg_quality
}

bool
subsea_interfaces__srv__CapturePair_Request__are_equal(const subsea_interfaces__srv__CapturePair_Request * lhs, const subsea_interfaces__srv__CapturePair_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // session_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->session_id), &(rhs->session_id)))
  {
    return false;
  }
  // output_dir
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->output_dir), &(rhs->output_dir)))
  {
    return false;
  }
  // jpeg_quality
  if (lhs->jpeg_quality != rhs->jpeg_quality) {
    return false;
  }
  return true;
}

bool
subsea_interfaces__srv__CapturePair_Request__copy(
  const subsea_interfaces__srv__CapturePair_Request * input,
  subsea_interfaces__srv__CapturePair_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // session_id
  if (!rosidl_runtime_c__String__copy(
      &(input->session_id), &(output->session_id)))
  {
    return false;
  }
  // output_dir
  if (!rosidl_runtime_c__String__copy(
      &(input->output_dir), &(output->output_dir)))
  {
    return false;
  }
  // jpeg_quality
  output->jpeg_quality = input->jpeg_quality;
  return true;
}

subsea_interfaces__srv__CapturePair_Request *
subsea_interfaces__srv__CapturePair_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  subsea_interfaces__srv__CapturePair_Request * msg = (subsea_interfaces__srv__CapturePair_Request *)allocator.allocate(sizeof(subsea_interfaces__srv__CapturePair_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(subsea_interfaces__srv__CapturePair_Request));
  bool success = subsea_interfaces__srv__CapturePair_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
subsea_interfaces__srv__CapturePair_Request__destroy(subsea_interfaces__srv__CapturePair_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    subsea_interfaces__srv__CapturePair_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
subsea_interfaces__srv__CapturePair_Request__Sequence__init(subsea_interfaces__srv__CapturePair_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  subsea_interfaces__srv__CapturePair_Request * data = NULL;

  if (size) {
    data = (subsea_interfaces__srv__CapturePair_Request *)allocator.zero_allocate(size, sizeof(subsea_interfaces__srv__CapturePair_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = subsea_interfaces__srv__CapturePair_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        subsea_interfaces__srv__CapturePair_Request__fini(&data[i - 1]);
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
subsea_interfaces__srv__CapturePair_Request__Sequence__fini(subsea_interfaces__srv__CapturePair_Request__Sequence * array)
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
      subsea_interfaces__srv__CapturePair_Request__fini(&array->data[i]);
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

subsea_interfaces__srv__CapturePair_Request__Sequence *
subsea_interfaces__srv__CapturePair_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  subsea_interfaces__srv__CapturePair_Request__Sequence * array = (subsea_interfaces__srv__CapturePair_Request__Sequence *)allocator.allocate(sizeof(subsea_interfaces__srv__CapturePair_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = subsea_interfaces__srv__CapturePair_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
subsea_interfaces__srv__CapturePair_Request__Sequence__destroy(subsea_interfaces__srv__CapturePair_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    subsea_interfaces__srv__CapturePair_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
subsea_interfaces__srv__CapturePair_Request__Sequence__are_equal(const subsea_interfaces__srv__CapturePair_Request__Sequence * lhs, const subsea_interfaces__srv__CapturePair_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!subsea_interfaces__srv__CapturePair_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
subsea_interfaces__srv__CapturePair_Request__Sequence__copy(
  const subsea_interfaces__srv__CapturePair_Request__Sequence * input,
  subsea_interfaces__srv__CapturePair_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(subsea_interfaces__srv__CapturePair_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    subsea_interfaces__srv__CapturePair_Request * data =
      (subsea_interfaces__srv__CapturePair_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!subsea_interfaces__srv__CapturePair_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          subsea_interfaces__srv__CapturePair_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!subsea_interfaces__srv__CapturePair_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `cam0_path`
// Member `cam1_path`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
subsea_interfaces__srv__CapturePair_Response__init(subsea_interfaces__srv__CapturePair_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    subsea_interfaces__srv__CapturePair_Response__fini(msg);
    return false;
  }
  // cam0_path
  if (!rosidl_runtime_c__String__init(&msg->cam0_path)) {
    subsea_interfaces__srv__CapturePair_Response__fini(msg);
    return false;
  }
  // cam1_path
  if (!rosidl_runtime_c__String__init(&msg->cam1_path)) {
    subsea_interfaces__srv__CapturePair_Response__fini(msg);
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    subsea_interfaces__srv__CapturePair_Response__fini(msg);
    return false;
  }
  return true;
}

void
subsea_interfaces__srv__CapturePair_Response__fini(subsea_interfaces__srv__CapturePair_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // cam0_path
  rosidl_runtime_c__String__fini(&msg->cam0_path);
  // cam1_path
  rosidl_runtime_c__String__fini(&msg->cam1_path);
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
subsea_interfaces__srv__CapturePair_Response__are_equal(const subsea_interfaces__srv__CapturePair_Response * lhs, const subsea_interfaces__srv__CapturePair_Response * rhs)
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
  // cam0_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cam0_path), &(rhs->cam0_path)))
  {
    return false;
  }
  // cam1_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cam1_path), &(rhs->cam1_path)))
  {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
subsea_interfaces__srv__CapturePair_Response__copy(
  const subsea_interfaces__srv__CapturePair_Response * input,
  subsea_interfaces__srv__CapturePair_Response * output)
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
  // cam0_path
  if (!rosidl_runtime_c__String__copy(
      &(input->cam0_path), &(output->cam0_path)))
  {
    return false;
  }
  // cam1_path
  if (!rosidl_runtime_c__String__copy(
      &(input->cam1_path), &(output->cam1_path)))
  {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

subsea_interfaces__srv__CapturePair_Response *
subsea_interfaces__srv__CapturePair_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  subsea_interfaces__srv__CapturePair_Response * msg = (subsea_interfaces__srv__CapturePair_Response *)allocator.allocate(sizeof(subsea_interfaces__srv__CapturePair_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(subsea_interfaces__srv__CapturePair_Response));
  bool success = subsea_interfaces__srv__CapturePair_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
subsea_interfaces__srv__CapturePair_Response__destroy(subsea_interfaces__srv__CapturePair_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    subsea_interfaces__srv__CapturePair_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
subsea_interfaces__srv__CapturePair_Response__Sequence__init(subsea_interfaces__srv__CapturePair_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  subsea_interfaces__srv__CapturePair_Response * data = NULL;

  if (size) {
    data = (subsea_interfaces__srv__CapturePair_Response *)allocator.zero_allocate(size, sizeof(subsea_interfaces__srv__CapturePair_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = subsea_interfaces__srv__CapturePair_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        subsea_interfaces__srv__CapturePair_Response__fini(&data[i - 1]);
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
subsea_interfaces__srv__CapturePair_Response__Sequence__fini(subsea_interfaces__srv__CapturePair_Response__Sequence * array)
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
      subsea_interfaces__srv__CapturePair_Response__fini(&array->data[i]);
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

subsea_interfaces__srv__CapturePair_Response__Sequence *
subsea_interfaces__srv__CapturePair_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  subsea_interfaces__srv__CapturePair_Response__Sequence * array = (subsea_interfaces__srv__CapturePair_Response__Sequence *)allocator.allocate(sizeof(subsea_interfaces__srv__CapturePair_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = subsea_interfaces__srv__CapturePair_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
subsea_interfaces__srv__CapturePair_Response__Sequence__destroy(subsea_interfaces__srv__CapturePair_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    subsea_interfaces__srv__CapturePair_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
subsea_interfaces__srv__CapturePair_Response__Sequence__are_equal(const subsea_interfaces__srv__CapturePair_Response__Sequence * lhs, const subsea_interfaces__srv__CapturePair_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!subsea_interfaces__srv__CapturePair_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
subsea_interfaces__srv__CapturePair_Response__Sequence__copy(
  const subsea_interfaces__srv__CapturePair_Response__Sequence * input,
  subsea_interfaces__srv__CapturePair_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(subsea_interfaces__srv__CapturePair_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    subsea_interfaces__srv__CapturePair_Response * data =
      (subsea_interfaces__srv__CapturePair_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!subsea_interfaces__srv__CapturePair_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          subsea_interfaces__srv__CapturePair_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!subsea_interfaces__srv__CapturePair_Response__copy(
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
// #include "subsea_interfaces/srv/detail/capture_pair__functions.h"

bool
subsea_interfaces__srv__CapturePair_Event__init(subsea_interfaces__srv__CapturePair_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    subsea_interfaces__srv__CapturePair_Event__fini(msg);
    return false;
  }
  // request
  if (!subsea_interfaces__srv__CapturePair_Request__Sequence__init(&msg->request, 0)) {
    subsea_interfaces__srv__CapturePair_Event__fini(msg);
    return false;
  }
  // response
  if (!subsea_interfaces__srv__CapturePair_Response__Sequence__init(&msg->response, 0)) {
    subsea_interfaces__srv__CapturePair_Event__fini(msg);
    return false;
  }
  return true;
}

void
subsea_interfaces__srv__CapturePair_Event__fini(subsea_interfaces__srv__CapturePair_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  subsea_interfaces__srv__CapturePair_Request__Sequence__fini(&msg->request);
  // response
  subsea_interfaces__srv__CapturePair_Response__Sequence__fini(&msg->response);
}

bool
subsea_interfaces__srv__CapturePair_Event__are_equal(const subsea_interfaces__srv__CapturePair_Event * lhs, const subsea_interfaces__srv__CapturePair_Event * rhs)
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
  if (!subsea_interfaces__srv__CapturePair_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!subsea_interfaces__srv__CapturePair_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
subsea_interfaces__srv__CapturePair_Event__copy(
  const subsea_interfaces__srv__CapturePair_Event * input,
  subsea_interfaces__srv__CapturePair_Event * output)
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
  if (!subsea_interfaces__srv__CapturePair_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!subsea_interfaces__srv__CapturePair_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

subsea_interfaces__srv__CapturePair_Event *
subsea_interfaces__srv__CapturePair_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  subsea_interfaces__srv__CapturePair_Event * msg = (subsea_interfaces__srv__CapturePair_Event *)allocator.allocate(sizeof(subsea_interfaces__srv__CapturePair_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(subsea_interfaces__srv__CapturePair_Event));
  bool success = subsea_interfaces__srv__CapturePair_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
subsea_interfaces__srv__CapturePair_Event__destroy(subsea_interfaces__srv__CapturePair_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    subsea_interfaces__srv__CapturePair_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
subsea_interfaces__srv__CapturePair_Event__Sequence__init(subsea_interfaces__srv__CapturePair_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  subsea_interfaces__srv__CapturePair_Event * data = NULL;

  if (size) {
    data = (subsea_interfaces__srv__CapturePair_Event *)allocator.zero_allocate(size, sizeof(subsea_interfaces__srv__CapturePair_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = subsea_interfaces__srv__CapturePair_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        subsea_interfaces__srv__CapturePair_Event__fini(&data[i - 1]);
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
subsea_interfaces__srv__CapturePair_Event__Sequence__fini(subsea_interfaces__srv__CapturePair_Event__Sequence * array)
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
      subsea_interfaces__srv__CapturePair_Event__fini(&array->data[i]);
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

subsea_interfaces__srv__CapturePair_Event__Sequence *
subsea_interfaces__srv__CapturePair_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  subsea_interfaces__srv__CapturePair_Event__Sequence * array = (subsea_interfaces__srv__CapturePair_Event__Sequence *)allocator.allocate(sizeof(subsea_interfaces__srv__CapturePair_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = subsea_interfaces__srv__CapturePair_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
subsea_interfaces__srv__CapturePair_Event__Sequence__destroy(subsea_interfaces__srv__CapturePair_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    subsea_interfaces__srv__CapturePair_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
subsea_interfaces__srv__CapturePair_Event__Sequence__are_equal(const subsea_interfaces__srv__CapturePair_Event__Sequence * lhs, const subsea_interfaces__srv__CapturePair_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!subsea_interfaces__srv__CapturePair_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
subsea_interfaces__srv__CapturePair_Event__Sequence__copy(
  const subsea_interfaces__srv__CapturePair_Event__Sequence * input,
  subsea_interfaces__srv__CapturePair_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(subsea_interfaces__srv__CapturePair_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    subsea_interfaces__srv__CapturePair_Event * data =
      (subsea_interfaces__srv__CapturePair_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!subsea_interfaces__srv__CapturePair_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          subsea_interfaces__srv__CapturePair_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!subsea_interfaces__srv__CapturePair_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
