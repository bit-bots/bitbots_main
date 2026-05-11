# Describes a detected marking intersection from vision modules, in image coordinates.
# An intersection is defined as a point where two or more line markings intersect.
# The term ray is used to describe the outgoing rays from the center of the
# intersection.
#
# For example, looking down at a perfectly upright T-intersection on the ground from above, you would have:
# - num_rays = 3
# - heading_rays = [0, pi / 2, pi]

# Center point of the intersection, in pixel coordinates
vision_msgs/Point2D center

# The number of rays outgoing from an intersection. This is 3 for a T-Junction and 
# 4 for an X-Junction
int32 num_rays

# NOTE: The length of the array should either:
# - Match the number of rays specified with num_rays OR
# - Be empty, if the headings are unknown
# Heading is defined as 0 along the +x axis, and pi / 2 along the +y axis.
# Headings should:
# - have an interval of [0, 2pi)
# - be stored in ascending order
float64[] heading_rays

soccer_vision_attribute_msgs/Confidence confidence
