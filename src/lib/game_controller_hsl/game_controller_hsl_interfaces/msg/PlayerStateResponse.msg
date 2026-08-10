# This message defines a PlayerStateResponse to be converted and sent as a RoboCupGameControlReturnData packet to the game controller.
# It is used to verify a player is connected to game controller and for visualization purposes.
#
# For the packet defition for RoboCupGameControlReturnData see:
# https://github.com/RoboCup-HumanoidSoccerLeague/GameController/blob/master/game_controller_msgs/headers/RoboCupGameControlData.h

# Time at which this response snapshot was assembled. frame_id is unused;
# pose and ball carry their own coordinate frames.
std_msgs/Header header

bool fallen

# This pose is based on the following field coordinate system:
#
#      y
#      ^       ______________________
#      |    M  |          |          |  O
#      |    Y  |_ -x, y   |   x, y  _|  P
#      |    G  | |        |        | |  P
# 0    +    O  | |       ( )       | |  G
#      |    A  |_|        |        |_|  O
#      |    L  |  -x,-y   |   x,-y   |  A
#      |       |__________|__________|  L
#      |
#      +------------------+--------------> x
#                         0
#
# 0,0 is the center of the field. pose.header.stamp is the time of the pose
# estimate and pose.header.frame_id identifies the field coordinate frame.
# Position is expressed in metres. The orientation represents yaw about the
# +z axis; it is converted to an angle in radians for the return packet, with
# yaw 0 along the +x axis and increasing counter-clockwise.
geometry_msgs/PoseStamped pose

# Ball position in metres, relative to the robot. ball.header.stamp is the time
# the ball was last observed and ball.header.frame_id identifies the applicable
# robot-relative coordinate frame, whose +x axis points forward and +y axis
# points left.
geometry_msgs/PointStamped ball
