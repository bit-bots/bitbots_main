import time

import math

import rospy
from humanoid_league_msgs.msg import HeadMode, BallInImage
import rosparam
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadCapsule:
    def __init__(self):

        self.config = rosparam.get_param("Behaviour/Head")
        self.delta = self.config["Search"]["headTurnPrecision"]
        self.wait_time = self.config["Search"]["headTurnTime"]
        self.pan_speed_max = self.config["Search"]["maxPanSpeedSearch"]
        self.tilt_speed_max = self.config["Search"]["maxTiltSpeedSearch"]

        # class variables
        self._headmode = 0
        self.confirmedBall = 0
        self.startedconfirmingball = 0
        self.confirmedGoal = 0
        self.startedconfirminggoal = 0
        self.current_pan_pos = 0
        self.current_tilt_pos = 0
        self.is_ball_tracking_still_active = False
        self.bestball_in_image = None, None

        # preparing message for more performance
        self.pos_msg = JointTrajectory()
        self.pos_msg.joint_names = ["HeadPan", "HeadTilt"]
        self.point_msg = JointTrajectoryPoint()
        self.pos_msg.points = [self.point_msg]

        self.position_publisher = None  # type: rospy.Publisher

    def send_motor_goals(self, pan_position: float, pan_speed: float, tilt_position: float, tilt_speed: float):
        rospy.loginfo("Move Head: %f, %f" % (pan_position, tilt_position))
        self.point_msg.positions = [math.radians(pan_position), math.radians(tilt_position)]
        self.point_msg.velocities = [pan_speed, tilt_speed]
        self.point_msg.time_from_start = rospy.Duration.from_sec(0.04)
        self.position_publisher.publish(self.pos_msg)

    def get_current_head_pos(self):
        return self.current_pan_pos, self.current_tilt_pos

    def get_headmode(self):
        return self._headmode

    def get_confirmed_ball(self):
        return self.confirmedBall

    def cb_headmode(self, headmode: HeadMode):
        self._headmode = headmode.headMode

    def joint_state_cb(self, msg: JointState):
        i = 0
        for joint in msg.name:
            if joint == "HeadPan":
                self.current_pan_pos = math.degrees(msg.position[i])
            elif joint == "HeadTilt":
                self.current_tilt_pos = math.degrees(msg.position[i])
            i += 1

    def cb_ballinimage(self, ball: BallInImage):
        self.bestball_in_image = ball.center.x, ball.center.y

    def get_started_confirm_ball(self):
        return self.startedconfirmingball

    def set_started_confirm_ball(self, t=time.time()):
        self.startedconfirmingball = t

    def unset_started_confirm_ball(self):
        self.startedconfirmingball = 0

    def set_confirmed_ball(self):
        self.startedconfirmingball = time.time()
        
    def get_started_confirm_goal(self):
        return self.startedconfirminggoal

    def set_started_confirm_goal(self, t=time.time()):
        self.startedconfirminggoal = t

    def unset_started_confirm_goal(self):
        self.startedconfirminggoal = 0

    def set_confirmed_goal(self):
        self.startedconfirminggoal = time.time()

    def cb_motor_postions(self, msg: JointTrajectory):
        self.pos_msg = msg
