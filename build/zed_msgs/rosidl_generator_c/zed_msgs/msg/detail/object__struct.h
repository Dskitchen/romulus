// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zed_msgs:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ZED_MSGS__MSG__DETAIL__OBJECT__STRUCT_H_
#define ZED_MSGS__MSG__DETAIL__OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'label'
// Member 'sublabel'
#include "rosidl_runtime_c/string.h"
// Member 'bounding_box_2d'
#include "zed_msgs/msg/detail/bounding_box2_di__struct.h"
// Member 'bounding_box_3d'
// Member 'head_bounding_box_3d'
#include "zed_msgs/msg/detail/bounding_box3_d__struct.h"
// Member 'head_bounding_box_2d'
#include "zed_msgs/msg/detail/bounding_box2_df__struct.h"
// Member 'skeleton_2d'
#include "zed_msgs/msg/detail/skeleton2_d__struct.h"
// Member 'skeleton_3d'
#include "zed_msgs/msg/detail/skeleton3_d__struct.h"

/// Struct defined in msg/Object in the package zed_msgs.
/**
  * Object label
 */
typedef struct zed_msgs__msg__Object
{
  rosidl_runtime_c__String label;
  /// Object label ID
  int16_t label_id;
  /// Object sub
  rosidl_runtime_c__String sublabel;
  /// Object confidence level (1-99)
  float confidence;
  /// Object centroid position
  float position[3];
  /// Position covariance
  float position_covariance[6];
  /// Object velocity
  float velocity[3];
  bool tracking_available;
  /// Tracking available
  /// Tracking state
  /// 0 -> OFF (object not valid)
  /// 1 -> OK
  /// 2 -> SEARCHING (occlusion occurred, trajectory is estimated)
  /// 3 -> TERMINATE (This is the last searching state of the track. The track will be deleted in the next frame).
  int8_t tracking_state;
  /// Action state
  /// 0 -> IDLE
  /// 2 -> MOVING
  int8_t action_state;
  /// 2D Bounding box projected to the Camera image
  zed_msgs__msg__BoundingBox2Di bounding_box_2d;
  /// 3D Bounding box in world frame
  zed_msgs__msg__BoundingBox3D bounding_box_3d;
  /// 3D dimensions (width, height, lenght)
  float dimensions_3d[3];
  /// Is skeleton available?
  bool skeleton_available;
  /// Skeleton format
  /// 0 -> POSE_18
  /// 1 -> POSE_34
  /// 2 -> POSE_38
  /// 3 -> POSE_70
  int8_t body_format;
  /// 2D Bounding box projected to Camera image of the person head
  zed_msgs__msg__BoundingBox2Df head_bounding_box_2d;
  /// 3D Bounding box in world frame of the person head
  zed_msgs__msg__BoundingBox3D head_bounding_box_3d;
  /// 3D position of the centroid of the person head
  float head_position[3];
  /// 2D Person skeleton projected to Camera image
  zed_msgs__msg__Skeleton2D skeleton_2d;
  /// 3D Person skeleton in world frame
  zed_msgs__msg__Skeleton3D skeleton_3d;
} zed_msgs__msg__Object;

// Struct for a sequence of zed_msgs__msg__Object.
typedef struct zed_msgs__msg__Object__Sequence
{
  zed_msgs__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zed_msgs__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZED_MSGS__MSG__DETAIL__OBJECT__STRUCT_H_
