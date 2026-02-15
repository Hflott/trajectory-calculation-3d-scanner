// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from subsea_interfaces:srv/CapturePair.idl
// generated code does not contain a copyright notice

#include "subsea_interfaces/srv/detail/capture_pair__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_subsea_interfaces
const rosidl_type_hash_t *
subsea_interfaces__srv__CapturePair__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x05, 0x08, 0x2f, 0xf7, 0x81, 0x03, 0xa7, 0x5b,
      0x14, 0x19, 0xfc, 0x7b, 0xfc, 0x43, 0x7c, 0x36,
      0x51, 0x40, 0x29, 0xae, 0xd8, 0x57, 0x59, 0xbb,
      0x1a, 0xd9, 0x1d, 0xef, 0xf0, 0xf3, 0xab, 0xb1,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_subsea_interfaces
const rosidl_type_hash_t *
subsea_interfaces__srv__CapturePair_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x20, 0xb8, 0x42, 0x35, 0xc6, 0x07, 0xf7, 0x1e,
      0xa3, 0x5f, 0x4b, 0xc9, 0xa1, 0x8a, 0x70, 0x26,
      0x30, 0x91, 0x5c, 0xd6, 0x32, 0xfa, 0x6f, 0x9a,
      0x31, 0xbb, 0x2a, 0x96, 0x04, 0xc7, 0x24, 0x2b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_subsea_interfaces
const rosidl_type_hash_t *
subsea_interfaces__srv__CapturePair_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x27, 0x84, 0x4b, 0xc4, 0xbe, 0x30, 0xfe, 0xf0,
      0xf8, 0xce, 0x9f, 0x7c, 0x0a, 0x95, 0x00, 0x0c,
      0xb2, 0xa0, 0xc4, 0x4e, 0x1c, 0xef, 0xbf, 0xee,
      0xaf, 0xb5, 0x9b, 0x61, 0xd2, 0xbf, 0xd2, 0xb7,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_subsea_interfaces
const rosidl_type_hash_t *
subsea_interfaces__srv__CapturePair_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe4, 0x53, 0x20, 0x38, 0xae, 0x02, 0xcc, 0x1c,
      0x47, 0xb0, 0xd9, 0x8e, 0xd0, 0x97, 0xe3, 0x5e,
      0xbb, 0x6a, 0x9c, 0xe3, 0xc2, 0x12, 0xcb, 0x1d,
      0xa5, 0x95, 0xe4, 0xa5, 0x66, 0x0e, 0x50, 0x63,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char subsea_interfaces__srv__CapturePair__TYPE_NAME[] = "subsea_interfaces/srv/CapturePair";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char subsea_interfaces__srv__CapturePair_Event__TYPE_NAME[] = "subsea_interfaces/srv/CapturePair_Event";
static char subsea_interfaces__srv__CapturePair_Request__TYPE_NAME[] = "subsea_interfaces/srv/CapturePair_Request";
static char subsea_interfaces__srv__CapturePair_Response__TYPE_NAME[] = "subsea_interfaces/srv/CapturePair_Response";

// Define type names, field names, and default values
static char subsea_interfaces__srv__CapturePair__FIELD_NAME__request_message[] = "request_message";
static char subsea_interfaces__srv__CapturePair__FIELD_NAME__response_message[] = "response_message";
static char subsea_interfaces__srv__CapturePair__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field subsea_interfaces__srv__CapturePair__FIELDS[] = {
  {
    {subsea_interfaces__srv__CapturePair__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {subsea_interfaces__srv__CapturePair_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {subsea_interfaces__srv__CapturePair_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {subsea_interfaces__srv__CapturePair_Event__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription subsea_interfaces__srv__CapturePair__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Event__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
subsea_interfaces__srv__CapturePair__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {subsea_interfaces__srv__CapturePair__TYPE_NAME, 33, 33},
      {subsea_interfaces__srv__CapturePair__FIELDS, 3, 3},
    },
    {subsea_interfaces__srv__CapturePair__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = subsea_interfaces__srv__CapturePair_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = subsea_interfaces__srv__CapturePair_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = subsea_interfaces__srv__CapturePair_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char subsea_interfaces__srv__CapturePair_Request__FIELD_NAME__session_id[] = "session_id";
static char subsea_interfaces__srv__CapturePair_Request__FIELD_NAME__output_dir[] = "output_dir";
static char subsea_interfaces__srv__CapturePair_Request__FIELD_NAME__jpeg_quality[] = "jpeg_quality";

static rosidl_runtime_c__type_description__Field subsea_interfaces__srv__CapturePair_Request__FIELDS[] = {
  {
    {subsea_interfaces__srv__CapturePair_Request__FIELD_NAME__session_id, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Request__FIELD_NAME__output_dir, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Request__FIELD_NAME__jpeg_quality, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
subsea_interfaces__srv__CapturePair_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {subsea_interfaces__srv__CapturePair_Request__TYPE_NAME, 41, 41},
      {subsea_interfaces__srv__CapturePair_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__success[] = "success";
static char subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__message[] = "message";
static char subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__cam0_path[] = "cam0_path";
static char subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__cam1_path[] = "cam1_path";
static char subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__stamp[] = "stamp";

static rosidl_runtime_c__type_description__Field subsea_interfaces__srv__CapturePair_Response__FIELDS[] = {
  {
    {subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__cam0_path, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__cam1_path, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Response__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription subsea_interfaces__srv__CapturePair_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
subsea_interfaces__srv__CapturePair_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {subsea_interfaces__srv__CapturePair_Response__TYPE_NAME, 42, 42},
      {subsea_interfaces__srv__CapturePair_Response__FIELDS, 5, 5},
    },
    {subsea_interfaces__srv__CapturePair_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char subsea_interfaces__srv__CapturePair_Event__FIELD_NAME__info[] = "info";
static char subsea_interfaces__srv__CapturePair_Event__FIELD_NAME__request[] = "request";
static char subsea_interfaces__srv__CapturePair_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field subsea_interfaces__srv__CapturePair_Event__FIELDS[] = {
  {
    {subsea_interfaces__srv__CapturePair_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {subsea_interfaces__srv__CapturePair_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {subsea_interfaces__srv__CapturePair_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription subsea_interfaces__srv__CapturePair_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {subsea_interfaces__srv__CapturePair_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
subsea_interfaces__srv__CapturePair_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {subsea_interfaces__srv__CapturePair_Event__TYPE_NAME, 39, 39},
      {subsea_interfaces__srv__CapturePair_Event__FIELDS, 3, 3},
    },
    {subsea_interfaces__srv__CapturePair_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = subsea_interfaces__srv__CapturePair_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = subsea_interfaces__srv__CapturePair_Response__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string session_id\n"
  "string output_dir\n"
  "int32 jpeg_quality\n"
  "---\n"
  "bool success\n"
  "string message\n"
  "string cam0_path\n"
  "string cam1_path\n"
  "builtin_interfaces/Time stamp";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
subsea_interfaces__srv__CapturePair__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {subsea_interfaces__srv__CapturePair__TYPE_NAME, 33, 33},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 151, 151},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
subsea_interfaces__srv__CapturePair_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {subsea_interfaces__srv__CapturePair_Request__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
subsea_interfaces__srv__CapturePair_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {subsea_interfaces__srv__CapturePair_Response__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
subsea_interfaces__srv__CapturePair_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {subsea_interfaces__srv__CapturePair_Event__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
subsea_interfaces__srv__CapturePair__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *subsea_interfaces__srv__CapturePair__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *subsea_interfaces__srv__CapturePair_Event__get_individual_type_description_source(NULL);
    sources[4] = *subsea_interfaces__srv__CapturePair_Request__get_individual_type_description_source(NULL);
    sources[5] = *subsea_interfaces__srv__CapturePair_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
subsea_interfaces__srv__CapturePair_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *subsea_interfaces__srv__CapturePair_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
subsea_interfaces__srv__CapturePair_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *subsea_interfaces__srv__CapturePair_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
subsea_interfaces__srv__CapturePair_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *subsea_interfaces__srv__CapturePair_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[3] = *subsea_interfaces__srv__CapturePair_Request__get_individual_type_description_source(NULL);
    sources[4] = *subsea_interfaces__srv__CapturePair_Response__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
