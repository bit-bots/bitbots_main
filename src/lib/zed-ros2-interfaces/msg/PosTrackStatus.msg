# CONSTANTS
uint8 OK = 0
uint8 UNAVAILABLE = 1
uint8 LOOP_CLOSED = 1
uint8 SEARCHING = 2
uint8 OFF = 3

# VISUAL_STATUS
# Represents the current state of Visual-Inertial Odometry (VIO) tracking between the previous frame and the current frame.
# OK  - The positional tracking module successfully tracked from the previous frame to the current frame.
# NOT_OK - The positional tracking module failed to track from the previous frame to the current frame.
uint8 odometry_status

# SPATIAL_MEMORY_STATUS
# Represents the current state of camera tracking in the global map.
# OK - The positional tracking module is operating normally.
# LOOP_CLOSED - The positional tracking module detected a loop and corrected its position.
# SEARCHING - The positional tracking module is searching for recognizable areas in the global map to relocate.
# OFF - Spatial memory is disabled
uint8 spatial_memory_status

# Deprecated
uint8 status
