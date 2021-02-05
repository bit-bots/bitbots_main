import rospy
import numpy
from humanoid_league_msgs.msg import GameState, RobotControlState

STATE_SHUT_DOWN = 0
STATE_STARTUP = 1
STATE_FALLEN = 2
STATE_NOTFALLEN = 3
STATE_NONGAME = 4
STATE_INGAME = 5
STATE_INIT = 6
STATE_SET = 7
STATE_PLAYING = 8
STATE_PENALTY = 9


class LocalizationBlackboard:

    def __init__(self):
        self.current_state = STATE_INIT
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
        self.last_gamestate_update_time = None
        self.game_state = None
        self.secondary_state = None
        self.first_half = None
        self.has_kickoff = None
        self.penalized = None
        self.secondsTillUnpenalized = None

        #Robot Control State
        self.robot_control_state = None


