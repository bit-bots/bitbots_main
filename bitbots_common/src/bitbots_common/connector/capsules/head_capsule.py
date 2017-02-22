import rospy
from humanoid_league_msgs.msg import HeadMode
import rosparam
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadCapsule:
    def __init__(self):

        self.config = rosparam.get_param("/Behaviour/Head")
        self.delta = self.config["Search"]["headTurnPrecision"]
        self.wait_time = self.config["Search"]["headTurnTime"]
        self.pan_speed_max = self.config["Search"]["maxPanSpeedSearch"]
        self.tilt_speed_max = self.config["Search"]["maxTiltSpeedSearch"]

        # class variables
        self.headmode = 0
        self.confirmedBall = 0
        self.confirmedGoal = 0
        self.current_pan_pos = 0
        self.current_tilt_pos = 0
        self.is_ball_tracking_still_active = False

        # preparing message for more performance
        self.pos_msg = JointTrajectory()
        self.pos_msg.joint_names = ["HeadPan", "HeadTilt"]
        self.point_msg = JointTrajectoryPoint()
        self.pos_msg.points = [self.point_msg]

        self.position_publisher = rospy.Publisher("/head_motor_goals", JointTrajectory, queue_size=10)
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

    def send_motor_goals(self, pan_position: float, pan_speed: float, tilt_position: float, tilt_speed: float):
        self.point_msg.positions = [pan_position, tilt_position]
        self.point_msg.velocities = [pan_speed, tilt_speed]
        self.position_publisher.publish(self.pos_msg)

    def get_current_head_pos(self):
        return self.current_pan_pos, self.current_tilt_pos

    def get_headmode(self):
        return self.headmode

    def get_confirmed_ball(self):
        return self.confirmedBall

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



