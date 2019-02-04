from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose2D
from std_msgs.msg import Header

import yaml
import rospy

from name import Name

class TrajectoryMsg:

    label_moveToX = "x"
    label_moveToY = "y"
    label_finalAngle = "an" # unused

    label_moveVelX = "vx"
    label_moveVelY = "vy"
    label_moveVelZ = "vz" # unused
    label_rotateVel = "van"

    title = "trajectory_msg"

    def __init__(self):
        self.data = {}

        # Labels
        #self.param_robot_id = "live_tool_id"

    def getMsg(self):
        # makes time stamp before sending udp package
        time = rospy.get_rostime()
        self.data[Name.timestamp] = {Name.secs: time.secs, Name.nsecs: time.nsecs}

        id = "NO_ID"
        if rospy.has_param(Name.param_robot_id):
            id = rospy.get_param(Name.param_robot_id)
        return id + "::" + TrajectoryMsg.title + "::" + yaml.dump(self.data)



    def setMoveBase(self, data):
        self.data[TrajectoryMsg.label_moveToX] = data.x
        self.data[TrajectoryMsg.label_moveToY] = data.y
        #self.data[TrajectoryMsg.label_finalAngle] = data.theta

    def setCmdVel(self, data):
        self.data[TrajectoryMsg.label_moveVelX] = data.linear.x
        self.data[TrajectoryMsg.label_moveVelY] = data.linear.y
        self.data[TrajectoryMsg.label_rotateVel] = data.angular.z
