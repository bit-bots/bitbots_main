# This Pose message is based on the following field coordinate system:
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
# 0,0 is the center of the filed and we expect (x, y, theta) with theta
# being the euler angle in radians, 0 along the +x axis,
# increasing counter clockwise

std_msgs/Header header

float32[3] pose
