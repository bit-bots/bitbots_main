from humanoid_league_msgs.msg import HeadMode
from sensor_msgs.msg import JointState


class HeadCapsule:
    def __init__(self):
        self.headmode = 0
        self.confirmedBall = 0
        self.confirmedGoal = 0
        self.current_pan_pos = 0
        self.current_tilt_pos = 0

    def get_headmode(self):
        return self.headmode

    def get_confirmed_ball(self):
        return self.confirmedBall

    def get_confirmed_goal(self):
        return self.confirmedGoal

    def cb_headmode(self, headmode:HeadMode):
        self.headmode = headmode.headMode

    def joint_state_cb(self, msg: JointState):
        i = 0
        for joint in msg.name:
            if joint == "HeadPan":
                self.current_pan_pos = msg.position[i]
            elif joint == "HeadTilt":
                self.current_tilt_pos = msg.position[i]
            i += 1



