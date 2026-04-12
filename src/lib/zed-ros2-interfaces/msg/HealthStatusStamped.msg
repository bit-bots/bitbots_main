# Message to provide information concerning the status of health of the ZED Camera

# Standard Header
std_msgs/Header header

# ZED Camera serial number
uint32 serial_number

# ZED Camera name
string camera_name

# Image quality is degraded
bool low_image_quality

# Environmental light conditions are not optimal
bool low_lighting

# Depth reliability is low
bool low_depth_reliability

# Motion sensors reliability is low
bool low_motion_sensors_reliability