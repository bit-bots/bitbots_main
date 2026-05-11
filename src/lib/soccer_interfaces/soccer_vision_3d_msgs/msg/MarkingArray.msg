# An array of markings with a header for global reference.
# This msg type is used to publish all markings detected in 3d space.

std_msgs/Header header
soccer_vision_3d_msgs/MarkingEllipse[] ellipses
soccer_vision_3d_msgs/MarkingIntersection[] intersections
soccer_vision_3d_msgs/MarkingSegment[] segments
