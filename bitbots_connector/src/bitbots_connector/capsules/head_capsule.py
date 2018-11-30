from humanoid_league_msgs.msg import HeadMode as HeadModeMsg

from bitbots_connector.blackboard import HeadBlackboard


class HeadCapsule:
    def __init__(self, blackboard):
        self.blackboard = blackboard

        # possible variables
        self.head_mode = None

    def head_mode_callback(self, msg: HeadModeMsg):
        self.head_mode = msg.headMode
