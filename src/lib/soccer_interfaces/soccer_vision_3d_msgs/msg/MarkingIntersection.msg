# Describes a detected marking intersection from vision modules, in 3d coordinates.
# An intersection is defined as a point where two or more line markings intersect.
# The term ray is used to describe the outgoing rays from the center of the
# intersection.

# Center point of the intersection
geometry_msgs/Point center 

# The number of rays outgoing from an intersection. This is 3 for a T-Junction and 
# 4 for an X-Junction
int32 num_rays

# NOTE: The length of the array should either:
# - Match the number of rays specified with num_rays OR
# - Be empty, if the headings are unknown
geometry_msgs/Vector3[] rays

soccer_vision_attribute_msgs/Confidence confidence
