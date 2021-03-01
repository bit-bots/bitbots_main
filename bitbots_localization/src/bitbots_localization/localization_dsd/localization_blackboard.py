import rospy
import numpy
from humanoid_league_msgs.msg import GameState, RobotControlState

class LocalizationBlackboard:

    def __init__(self):
        self.current_time = rospy.Time()
        self.shut_down_request = False
        self.last_initialized = None
        self.initialized = False

        # Pose
        self.pose_timeout_duration = rospy.Time(10)
        self.last_pose_update_time = None
        self.poseX = 0
        self.poseY = 0
        self.orientation = numpy.array([0, 0, 0, 1])
        self.covariance = numpy.array([])

        #GameState
        self.game_state_received = False
        self.game_state = None
        self.secondary_state = None
        self.first_half = None
        self.has_kickoff = None
        self.penalized = None
        self.secondsTillUnpenalized = None

        #Robot Control State
        self.robot_control_state = None
        self.last_robot_control_state = None

        #Get up
        self.last_state_get_up = False

        #Picked up
        self.last_state_pickup = False
        
