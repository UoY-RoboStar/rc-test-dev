// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sample_interface:msg/Input.idl
// generated code does not contain a copyright notice

#ifndef SAMPLE_INTERFACE__MSG__DETAIL__INPUT__FUNCTIONS_H_
#define SAMPLE_INTERFACE__MSG__DETAIL__INPUT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "sample_interface/msg/rosidl_generator_c__visibility_control.h"

#include "sample_interface/msg/detail/input__struct.h"

/// Initialize msg/Input message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sample_interface__msg__Input
 * )) before or use
 * sample_interface__msg__Input__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
bool
sample_interface__msg__Input__init(sample_interface__msg__Input * msg);

/// Finalize msg/Input message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
void
sample_interface__msg__Input__fini(sample_interface__msg__Input * msg);

/// Create msg/Input message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sample_interface__msg__Input__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
sample_interface__msg__Input *
sample_interface__msg__Input__create();

/// Destroy msg/Input message.
/**
 * It calls
 * sample_interface__msg__Input__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
void
sample_interface__msg__Input__destroy(sample_interface__msg__Input * msg);

/// Check for msg/Input message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
bool
sample_interface__msg__Input__are_equal(const sample_interface__msg__Input * lhs, const sample_interface__msg__Input * rhs);

/// Copy a msg/Input message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
bool
sample_interface__msg__Input__copy(
  const sample_interface__msg__Input * input,
  sample_interface__msg__Input * output);

/// Initialize array of msg/Input messages.
/**
 * It allocates the memory for the number of elements and calls
 * sample_interface__msg__Input__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
bool
sample_interface__msg__Input__Sequence__init(sample_interface__msg__Input__Sequence * array, size_t size);

/// Finalize array of msg/Input messages.
/**
 * It calls
 * sample_interface__msg__Input__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
void
sample_interface__msg__Input__Sequence__fini(sample_interface__msg__Input__Sequence * array);

/// Create array of msg/Input messages.
/**
 * It allocates the memory for the array and calls
 * sample_interface__msg__Input__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
sample_interface__msg__Input__Sequence *
sample_interface__msg__Input__Sequence__create(size_t size);

/// Destroy array of msg/Input messages.
/**
 * It calls
 * sample_interface__msg__Input__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
void
sample_interface__msg__Input__Sequence__destroy(sample_interface__msg__Input__Sequence * array);

/// Check for msg/Input message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
bool
sample_interface__msg__Input__Sequence__are_equal(const sample_interface__msg__Input__Sequence * lhs, const sample_interface__msg__Input__Sequence * rhs);

/// Copy an array of msg/Input messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sample_interface
bool
sample_interface__msg__Input__Sequence__copy(
  const sample_interface__msg__Input__Sequence * input,
  sample_interface__msg__Input__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SAMPLE_INTERFACE__MSG__DETAIL__INPUT__FUNCTIONS_H_
