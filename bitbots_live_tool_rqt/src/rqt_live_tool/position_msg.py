from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose
from std_msgs.msg import Header
import yaml
import rospy
import tf

from name import Name

class PositionMsg:

    label_orientation = "o"
    label_pos = "p"
    label_yaw = "yw"

    title = "position_msg"

    def __init__(self):

        # the dict containing message data
        self.data = {}

        #self.param_robot_id = "live_tool_id"

        # labels for dict
        #self.s_position = "pos"
        #self.s_orientation = "orientation"


    # Takes a PoseWithCovarianceStamped Msg and writes necessary information in data
    def setPoseWithCovarianceStamped(self, data):
        """
        dictionary with information about the position of the robot
        :param data: a dictionary with the transmitted information
        :return:
        """
        self.data[Name.last_update] = data.header.stamp.secs

        self.data[PositionMsg.label_pos] = {"x": data.pose.pose.position.x,
                                      "y": data.pose.pose.position.y}
                                      #"z": data.pose.pose.position.z}

        quat = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        #roll = euler[0]
        #pitch = euler[1]
        yaw = euler[2]

        self.data[PositionMsg.label_orientation] = {PositionMsg.label_yaw: yaw} #,
                                         #"pitch": pitch,
                                         #"roll": roll}

    # Returns yaml string containing relevant information
    def getMsg(self):
        """

        :return:
        """
        # makes time stamp before sending udp package
        time = rospy.get_rostime()
        self.data[Name.timestamp] = {Name.secs: time.secs, Name.nsecs: time.nsecs}

        id = "NO_ID"
        if rospy.has_param( Name.param_robot_id ):
            id = rospy.get_param(Name.param_robot_id)

        return id + "::" +  PositionMsg.title + "::" + yaml.dump(self.data)

