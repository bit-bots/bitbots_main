from humanoid_league_msgs.msg import HeadMode as HeadModeMsg



class HeadCapsule:
    def __init__(self, blackboard):
        self.blackboard = blackboard

        # possible variables
        self.head_mode = None

    def head_mode_callback(self, msg):
        self.head_mode = msg.headMode
