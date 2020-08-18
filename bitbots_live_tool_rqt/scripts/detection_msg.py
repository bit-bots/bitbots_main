from humanoid_league_msgs.msg import BallRelative, ObstacleRelative
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header
import yaml
import rospy

from name import Name

class DetectionMsg:

    label_obstacles = "o"
    label_obstacle_pos = "op"
    label_obstacle_info = "oi"
    label_color = "c"
    label_last_obstacle_update = "lo"
    #label_player_nr = "playernr"
    #label_width = "width"
    #label_height = "height"
    #label_confidence = "confidence"

    label_ball_info = "bi"

    title = "detection_msg"

    def __init__(self):

        # detection dictionary with information about the detection
        self.data = {}

        # Label for Ball Information
        #self.ballTime = "ball_timestamp"
        #self.position = "ball_position"

        # Label for Obstacle Stuff
        #self.obstacles = "obstacles"
        #self.obPosition = "position"
        #self.obs_info = "info"

        # Label for Goal Information
        #self.goalPostRight = "goal_post_right"
        #self.goalPostLeft = "goal_post_left"
        #self.goalCenterDirection = "goal_center_direction"
        #self.goalTimestamp = "goal_timestamp"
        #self.param_robot_id = "live_tool_id"

    def getMsg(self):
        """
        gets the most recent message
        :return:
        """
        # makes time stamp before sending udp package
        time = rospy.get_rostime()
        self.data[Name.timestamp] = {Name.secs: time.secs, Name.nsecs: time.nsecs}

        id = "NO_ID"
        if rospy.has_param(Name.param_robot_id):
            id = rospy.get_param(Name.param_robot_id)

        return id + "::" + DetectionMsg.title + "::" + yaml.dump(self.data)

    # Set Informations for Ball
    def setBallRelative(self, data):
        """
        fills the dictionary with information of the ball
        :param data:a dictionary with the transmitted information
        :return:
        """
        #self.data[self.ballTime] = {Name.secs: data.header.stamp.secs,
        #                            Name.nsecs: data.header.stamp.nsecs}
        time = rospy.get_rostime()

        self.data[DetectionMsg.label_ball_info] = {"x": data.ball_relative.x,\
                                                   "y": data.ball_relative.y,\
                                                   "lu" : time.secs}
                                    #"z": data.ball_relative.z,
                                    #"conf": data.confidence}

    # set Informations for Obstalce
    def setObstacleRelative(self, data):
        """
        fills the dictionary with information of the obstacles
        :param data: a dictionary with the transmitted information
        :return:
        """
        # an obstacle list which contains dictionary with obstacle information
        obstacles = []
        for obs in data.obstacles:
            #print("here obs: " + str(obs))
            obsentry = {}
            obsentry[DetectionMsg.label_obstacle_pos] = {"x": obs.pose.pose.pose.position.x,\
                                        "y": obs.pose.pose.pose.position.y} #,\
                                        #"z": obs.position.z}

            obsentry[DetectionMsg.label_obstacle_info] = {DetectionMsg.label_color: obs.type}#,
                                                          #DetectionMsg.label_player_nr: obs.playerNumber,
                                                          #DetectionMsg.label_width: obs.width,
                                                          #DetectionMsg.label_height: obs.height,
                                                          #DetectionMsg.label_confidence: obs.confidence}
            obstacles.append(obsentry)
        self.data[DetectionMsg.label_obstacles] = obstacles
        time = rospy.get_rostime()
        self.data[DetectionMsg.label_last_obstacle_update] = time.secs

    """
    def setGoalRelative(self, data):

        fills the dictionary with information of the goal
        :param data: a dictionary with the transmitted information
        :return:

        self.data[self.goalPostLeft] = {"x": data.left_post.x,
                                        "y": data.left_post.y,
                                        "z": data.left_post.z}
        self.data[self.goalPostRight] = {"x": data.right_post.x,
                                         "y": data.right_post.y,
                                         "z": data.right_post.z}
        self.data[self.goalCenterDirection] = {"x": data.center_direction.x,
                                               "y": data.center_direction.y,
                                               "z": data.center_direction.z}
        self.data[self.goalTimestamp] = {Name.secs: data.header.stamp.secs,
                                         Name.nsecs: data.header.stamp.nsecs}
    """
