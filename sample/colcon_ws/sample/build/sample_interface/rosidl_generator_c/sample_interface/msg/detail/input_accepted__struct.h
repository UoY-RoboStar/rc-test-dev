// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interface:msg/InputAccepted.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__STRUCT_H_
#define SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/InputAccepted in the package sample_interface.
typedef struct sample_interface__msg__InputAccepted
{
  builtin_interfaces__msg__Time stamp;
} sample_interface__msg__InputAccepted;

// Struct for a sequence of sample_interface__msg__InputAccepted.
typedef struct sample_interface__msg__InputAccepted__Sequence
{
  sample_interface__msg__InputAccepted * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interface__msg__InputAccepted__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__INPUT_ACCEPTED__STRUCT_H_
