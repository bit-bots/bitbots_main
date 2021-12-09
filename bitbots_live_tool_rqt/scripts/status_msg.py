from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose
from humanoid_league_msgs.msg import GameState, Strategy, RobotControlState
import yaml
import rospy


from name import Name

class StatusMsg:

    labelTeamColor = "tmc"
    label_first_half = "fh"
    label_own_score = "os"
    label_rival_score = "rs"
    label_remain_secs = "s"
    label_penalized = "pn"
    label_penalty_rest = "pr"

    label_role = "rl"
    label_action = "ac"

    label_state = "st"

    title = "status_msg"

    def __init__(self):

        #self.param_robot_id = "live_tool_id"

        # the dict containing message data
        self.data = {}


    # Takes a PoseWithCovarianceStamped Msg and writes necessary information in data
    def setGameState(self, data):

        #self.data[Name.timestamp] = {Name.secs: data.header.stamp.secs, Name.nsecs: data.header.stamp.nsecs}

        """
        self.data["GAMESTATE_INITAL"] = data.GAMESTATE_INITAL
        self.data["GAMESTATE_READY"] = data.GAMESTATE_READY
        self.data["GAMESTATE_SET"] = data.GAMESTATE_SET
        self.data["GAMESTATE_PLAYING"] = data.GAMESTATE_PLAYING
        self.data["GAMESTATE_FINISHED"] = data.GAMESTATE_FINISHED
        self.data["STATE_NORMAL"] = data.STATE_NORMAL
        self.data["STATE_PENALTYSHOOT"] = data.STATE_PENALTYSHOOT
        self.data["STATE_OVERTIME"] = data.STATE_OVERTIME
        self.data["STATE_TIMEOUT"] = data.STATE_TIMEOUT
        self.data["STATE_DIRECT_FREEKICK"] = data.STATE_DIRECT_FREEKICK
        self.data["STATE_INDIRECT_FREEKICK"] = data.STATE_INDIRECT_FREEKICK
        self.data["STATE_PENALTYKICK"] = data.STATE_PENALTYKICK
        self.data["BLUE"] = data.BLUE 0
        self.data["RED"] = data.RED 1
        """

        #self.data["gamestate"] = data.gameState
        #self.data["sec_state"] = data.secondaryState
        #self.data["sec_state_team"] = data.secondaryStateTeam
        self.data[StatusMsg.label_first_half] = data.firstHalf
        self.data[StatusMsg.label_own_score] = data.ownScore
        self.data[StatusMsg.label_rival_score] = data.rivalScore
        self.data[StatusMsg.label_remain_secs] = data.secondsRemaining
        #self.data["remain_scnd_secs"] = data.secondary_seconds_remaining
        self.data[StatusMsg.label_penalized] = data.penalized #for green and red indicator; active , inactive rob
        self.data[StatusMsg.label_penalty_rest] = data.secondsTillUnpenalized
        #self.data["move_allow"] = data.allowedToMove
        self.data[StatusMsg.labelTeamColor] = data.teamColor
        #self.data["penalty_shot"] = data.penaltyShot
        #self.data["single_shot"] = data.singleShots


    def setStrategy(self, data):
        """

        :param data:a dictionary with the transmitted information
        :return:

        uint8 ROLE_UNDEFINED=0
        uint8 ROLE_IDLING=1
        uint8 ROLE_OTHER=2
        uint8 ROLE_STRIKER=3
        uint8 ROLE_SUPPORTER=4
        uint8 ROLE_DEFENDER=5
        uint8 ROLE_GOALIE=6
        uint8 ACTION_UNDEFINED=0
        uint8 ACTION_POSITIONING=1
        uint8 ACTION_GOING_TO_BALL=2
        uint8 ACTION_TRYING_TO_SCORE=3
        uint8 ACTION_WAITING=4
        uint8 SIDE_UNDEFINED=0
        uint8 SIDE_LEFT=1
        uint8 SIDE_MIDDLE=2
        uint8 SIDE_RIGHT=3
        """

        self.data[StatusMsg.label_role] = data.role
        self.data[StatusMsg.label_action] = data.action
        #self.data["offensive_side"] = data.offensive_side

    def setStatusMsg(self, data):
        self.data[StatusMsg.label_state] = data.state
        """
        uint8 CONTROLLABLE=0
        uint8 FALLING=1
        uint8 FALLEN=2
        uint8 GETTING_UP=3
        uint8 ANIMATION_RUNNING=4
        uint8 STARTUP=5
        uint8 SHUTDOWN=6
        uint8 PENALTY=7
        uint8 PENALTY_ANIMANTION=8
        uint8 RECORD=9
        uint8 WALKING=10
        uint8 MOTOR_OFF=11
        uint8 HCM_OFF=12
        uint8 HARDWARE_PROBLEM=13
        uint8 PICKED_UP=14"""


    # Returns yaml string containing relevant information
    def getMsg(self):
        # makes time stamp before sending udp package
        time = rospy.get_rostime()
        self.data[Name.timestamp] = {Name.secs: time.secs, Name.nsecs: time.nsecs}

        id = "NO_ID"
        if rospy.has_param( Name.param_robot_id ):
            id = rospy.get_param(Name.param_robot_id)

        return id + "::" +  StatusMsg.title + "::" + yaml.dump(self.data)
