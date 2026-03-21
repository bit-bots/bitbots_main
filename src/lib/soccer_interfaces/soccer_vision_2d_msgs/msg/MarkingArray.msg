# An array of markings with a header for global reference.
# This msg type is used to publish all markings detected in an image.

std_msgs/Header header
soccer_vision_2d_msgs/MarkingEllipse[] ellipses
soccer_vision_2d_msgs/MarkingIntersection[] intersections
soccer_vision_2d_msgs/MarkingSegment[] segments
