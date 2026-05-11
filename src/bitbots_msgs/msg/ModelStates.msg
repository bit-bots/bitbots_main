# Originally from the now deprecated gazebo msgs (we used it for webots and do not really care about gazebo).
# Can be used for ground truth data from simulation.

string[] name                 # model names
geometry_msgs/Pose[] pose     # desired pose in world frame
geometry_msgs/Twist[] twist   # desired twist in world frame
