// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sample_interface:msg/Input.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__INPUT__STRUCT_H_
#define SAMPLE_INTERFACE__MSG__DETAIL__INPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Input in the package sample_interface.
typedef struct sample_interface__msg__Input
{
  uint32_t value;
} sample_interface__msg__Input;

// Struct for a sequence of sample_interface__msg__Input.
typedef struct sample_interface__msg__Input__Sequence
{
  sample_interface__msg__Input * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sample_interface__msg__Input__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__INPUT__STRUCT_H_
