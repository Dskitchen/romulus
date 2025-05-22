# PatternPoints.msg
# A message to represent a list of 2D points in image coordinates (pixels) for overlaying patterns on an image.
std_msgs/Header header  # Timestamp and frame_id for synchronization with image data

geometry_msgs/Point32[] points  # List of points in image coordinates (pixels)
# - points[i].x: u-coordinate (horizontal pixel position)
# - points[i].y: v-coordinate (vertical pixel position)
# - points[i].z: Unused (set to 0.0)
