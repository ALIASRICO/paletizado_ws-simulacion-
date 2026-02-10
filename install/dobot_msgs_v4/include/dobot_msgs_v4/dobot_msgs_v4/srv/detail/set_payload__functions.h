// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dobot_msgs_v4:srv/SetPayload.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dobot_msgs_v4/srv/set_payload.h"


#ifndef DOBOT_MSGS_V4__SRV__DETAIL__SET_PAYLOAD__FUNCTIONS_H_
#define DOBOT_MSGS_V4__SRV__DETAIL__SET_PAYLOAD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "dobot_msgs_v4/msg/rosidl_generator_c__visibility_control.h"

#include "dobot_msgs_v4/srv/detail/set_payload__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__SetPayload__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__SetPayload__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__SetPayload__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__SetPayload__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/SetPayload message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dobot_msgs_v4__srv__SetPayload_Request
 * )) before or use
 * dobot_msgs_v4__srv__SetPayload_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Request__init(dobot_msgs_v4__srv__SetPayload_Request * msg);

/// Finalize srv/SetPayload message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Request__fini(dobot_msgs_v4__srv__SetPayload_Request * msg);

/// Create srv/SetPayload message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dobot_msgs_v4__srv__SetPayload_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
dobot_msgs_v4__srv__SetPayload_Request *
dobot_msgs_v4__srv__SetPayload_Request__create(void);

/// Destroy srv/SetPayload message.
/**
 * It calls
 * dobot_msgs_v4__srv__SetPayload_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Request__destroy(dobot_msgs_v4__srv__SetPayload_Request * msg);

/// Check for srv/SetPayload message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Request__are_equal(const dobot_msgs_v4__srv__SetPayload_Request * lhs, const dobot_msgs_v4__srv__SetPayload_Request * rhs);

/// Copy a srv/SetPayload message.
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
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Request__copy(
  const dobot_msgs_v4__srv__SetPayload_Request * input,
  dobot_msgs_v4__srv__SetPayload_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__SetPayload_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__SetPayload_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__SetPayload_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__SetPayload_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/SetPayload messages.
/**
 * It allocates the memory for the number of elements and calls
 * dobot_msgs_v4__srv__SetPayload_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Request__Sequence__init(dobot_msgs_v4__srv__SetPayload_Request__Sequence * array, size_t size);

/// Finalize array of srv/SetPayload messages.
/**
 * It calls
 * dobot_msgs_v4__srv__SetPayload_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Request__Sequence__fini(dobot_msgs_v4__srv__SetPayload_Request__Sequence * array);

/// Create array of srv/SetPayload messages.
/**
 * It allocates the memory for the array and calls
 * dobot_msgs_v4__srv__SetPayload_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
dobot_msgs_v4__srv__SetPayload_Request__Sequence *
dobot_msgs_v4__srv__SetPayload_Request__Sequence__create(size_t size);

/// Destroy array of srv/SetPayload messages.
/**
 * It calls
 * dobot_msgs_v4__srv__SetPayload_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Request__Sequence__destroy(dobot_msgs_v4__srv__SetPayload_Request__Sequence * array);

/// Check for srv/SetPayload message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Request__Sequence__are_equal(const dobot_msgs_v4__srv__SetPayload_Request__Sequence * lhs, const dobot_msgs_v4__srv__SetPayload_Request__Sequence * rhs);

/// Copy an array of srv/SetPayload messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Request__Sequence__copy(
  const dobot_msgs_v4__srv__SetPayload_Request__Sequence * input,
  dobot_msgs_v4__srv__SetPayload_Request__Sequence * output);

/// Initialize srv/SetPayload message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dobot_msgs_v4__srv__SetPayload_Response
 * )) before or use
 * dobot_msgs_v4__srv__SetPayload_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Response__init(dobot_msgs_v4__srv__SetPayload_Response * msg);

/// Finalize srv/SetPayload message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Response__fini(dobot_msgs_v4__srv__SetPayload_Response * msg);

/// Create srv/SetPayload message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dobot_msgs_v4__srv__SetPayload_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
dobot_msgs_v4__srv__SetPayload_Response *
dobot_msgs_v4__srv__SetPayload_Response__create(void);

/// Destroy srv/SetPayload message.
/**
 * It calls
 * dobot_msgs_v4__srv__SetPayload_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Response__destroy(dobot_msgs_v4__srv__SetPayload_Response * msg);

/// Check for srv/SetPayload message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Response__are_equal(const dobot_msgs_v4__srv__SetPayload_Response * lhs, const dobot_msgs_v4__srv__SetPayload_Response * rhs);

/// Copy a srv/SetPayload message.
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
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Response__copy(
  const dobot_msgs_v4__srv__SetPayload_Response * input,
  dobot_msgs_v4__srv__SetPayload_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__SetPayload_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__SetPayload_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__SetPayload_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__SetPayload_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/SetPayload messages.
/**
 * It allocates the memory for the number of elements and calls
 * dobot_msgs_v4__srv__SetPayload_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Response__Sequence__init(dobot_msgs_v4__srv__SetPayload_Response__Sequence * array, size_t size);

/// Finalize array of srv/SetPayload messages.
/**
 * It calls
 * dobot_msgs_v4__srv__SetPayload_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Response__Sequence__fini(dobot_msgs_v4__srv__SetPayload_Response__Sequence * array);

/// Create array of srv/SetPayload messages.
/**
 * It allocates the memory for the array and calls
 * dobot_msgs_v4__srv__SetPayload_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
dobot_msgs_v4__srv__SetPayload_Response__Sequence *
dobot_msgs_v4__srv__SetPayload_Response__Sequence__create(size_t size);

/// Destroy array of srv/SetPayload messages.
/**
 * It calls
 * dobot_msgs_v4__srv__SetPayload_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Response__Sequence__destroy(dobot_msgs_v4__srv__SetPayload_Response__Sequence * array);

/// Check for srv/SetPayload message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Response__Sequence__are_equal(const dobot_msgs_v4__srv__SetPayload_Response__Sequence * lhs, const dobot_msgs_v4__srv__SetPayload_Response__Sequence * rhs);

/// Copy an array of srv/SetPayload messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Response__Sequence__copy(
  const dobot_msgs_v4__srv__SetPayload_Response__Sequence * input,
  dobot_msgs_v4__srv__SetPayload_Response__Sequence * output);

/// Initialize srv/SetPayload message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dobot_msgs_v4__srv__SetPayload_Event
 * )) before or use
 * dobot_msgs_v4__srv__SetPayload_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Event__init(dobot_msgs_v4__srv__SetPayload_Event * msg);

/// Finalize srv/SetPayload message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Event__fini(dobot_msgs_v4__srv__SetPayload_Event * msg);

/// Create srv/SetPayload message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dobot_msgs_v4__srv__SetPayload_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
dobot_msgs_v4__srv__SetPayload_Event *
dobot_msgs_v4__srv__SetPayload_Event__create(void);

/// Destroy srv/SetPayload message.
/**
 * It calls
 * dobot_msgs_v4__srv__SetPayload_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Event__destroy(dobot_msgs_v4__srv__SetPayload_Event * msg);

/// Check for srv/SetPayload message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Event__are_equal(const dobot_msgs_v4__srv__SetPayload_Event * lhs, const dobot_msgs_v4__srv__SetPayload_Event * rhs);

/// Copy a srv/SetPayload message.
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
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Event__copy(
  const dobot_msgs_v4__srv__SetPayload_Event * input,
  dobot_msgs_v4__srv__SetPayload_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_type_hash_t *
dobot_msgs_v4__srv__SetPayload_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeDescription *
dobot_msgs_v4__srv__SetPayload_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeSource *
dobot_msgs_v4__srv__SetPayload_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
const rosidl_runtime_c__type_description__TypeSource__Sequence *
dobot_msgs_v4__srv__SetPayload_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/SetPayload messages.
/**
 * It allocates the memory for the number of elements and calls
 * dobot_msgs_v4__srv__SetPayload_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Event__Sequence__init(dobot_msgs_v4__srv__SetPayload_Event__Sequence * array, size_t size);

/// Finalize array of srv/SetPayload messages.
/**
 * It calls
 * dobot_msgs_v4__srv__SetPayload_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Event__Sequence__fini(dobot_msgs_v4__srv__SetPayload_Event__Sequence * array);

/// Create array of srv/SetPayload messages.
/**
 * It allocates the memory for the array and calls
 * dobot_msgs_v4__srv__SetPayload_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
dobot_msgs_v4__srv__SetPayload_Event__Sequence *
dobot_msgs_v4__srv__SetPayload_Event__Sequence__create(size_t size);

/// Destroy array of srv/SetPayload messages.
/**
 * It calls
 * dobot_msgs_v4__srv__SetPayload_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
void
dobot_msgs_v4__srv__SetPayload_Event__Sequence__destroy(dobot_msgs_v4__srv__SetPayload_Event__Sequence * array);

/// Check for srv/SetPayload message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Event__Sequence__are_equal(const dobot_msgs_v4__srv__SetPayload_Event__Sequence * lhs, const dobot_msgs_v4__srv__SetPayload_Event__Sequence * rhs);

/// Copy an array of srv/SetPayload messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dobot_msgs_v4
bool
dobot_msgs_v4__srv__SetPayload_Event__Sequence__copy(
  const dobot_msgs_v4__srv__SetPayload_Event__Sequence * input,
  dobot_msgs_v4__srv__SetPayload_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // DOBOT_MSGS_V4__SRV__DETAIL__SET_PAYLOAD__FUNCTIONS_H_
