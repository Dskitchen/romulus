// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from zed_msgs:msg/GnssFusionStatus.idl
// generated code does not contain a copyright notice

#ifndef ZED_MSGS__MSG__DETAIL__GNSS_FUSION_STATUS__FUNCTIONS_H_
#define ZED_MSGS__MSG__DETAIL__GNSS_FUSION_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "zed_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "zed_msgs/msg/detail/gnss_fusion_status__struct.h"

/// Initialize msg/GnssFusionStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * zed_msgs__msg__GnssFusionStatus
 * )) before or use
 * zed_msgs__msg__GnssFusionStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
bool
zed_msgs__msg__GnssFusionStatus__init(zed_msgs__msg__GnssFusionStatus * msg);

/// Finalize msg/GnssFusionStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
void
zed_msgs__msg__GnssFusionStatus__fini(zed_msgs__msg__GnssFusionStatus * msg);

/// Create msg/GnssFusionStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * zed_msgs__msg__GnssFusionStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
zed_msgs__msg__GnssFusionStatus *
zed_msgs__msg__GnssFusionStatus__create();

/// Destroy msg/GnssFusionStatus message.
/**
 * It calls
 * zed_msgs__msg__GnssFusionStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
void
zed_msgs__msg__GnssFusionStatus__destroy(zed_msgs__msg__GnssFusionStatus * msg);

/// Check for msg/GnssFusionStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
bool
zed_msgs__msg__GnssFusionStatus__are_equal(const zed_msgs__msg__GnssFusionStatus * lhs, const zed_msgs__msg__GnssFusionStatus * rhs);

/// Copy a msg/GnssFusionStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
bool
zed_msgs__msg__GnssFusionStatus__copy(
  const zed_msgs__msg__GnssFusionStatus * input,
  zed_msgs__msg__GnssFusionStatus * output);

/// Initialize array of msg/GnssFusionStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * zed_msgs__msg__GnssFusionStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
bool
zed_msgs__msg__GnssFusionStatus__Sequence__init(zed_msgs__msg__GnssFusionStatus__Sequence * array, size_t size);

/// Finalize array of msg/GnssFusionStatus messages.
/**
 * It calls
 * zed_msgs__msg__GnssFusionStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
void
zed_msgs__msg__GnssFusionStatus__Sequence__fini(zed_msgs__msg__GnssFusionStatus__Sequence * array);

/// Create array of msg/GnssFusionStatus messages.
/**
 * It allocates the memory for the array and calls
 * zed_msgs__msg__GnssFusionStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
zed_msgs__msg__GnssFusionStatus__Sequence *
zed_msgs__msg__GnssFusionStatus__Sequence__create(size_t size);

/// Destroy array of msg/GnssFusionStatus messages.
/**
 * It calls
 * zed_msgs__msg__GnssFusionStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
void
zed_msgs__msg__GnssFusionStatus__Sequence__destroy(zed_msgs__msg__GnssFusionStatus__Sequence * array);

/// Check for msg/GnssFusionStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
bool
zed_msgs__msg__GnssFusionStatus__Sequence__are_equal(const zed_msgs__msg__GnssFusionStatus__Sequence * lhs, const zed_msgs__msg__GnssFusionStatus__Sequence * rhs);

/// Copy an array of msg/GnssFusionStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_zed_msgs
bool
zed_msgs__msg__GnssFusionStatus__Sequence__copy(
  const zed_msgs__msg__GnssFusionStatus__Sequence * input,
  zed_msgs__msg__GnssFusionStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ZED_MSGS__MSG__DETAIL__GNSS_FUSION_STATUS__FUNCTIONS_H_
