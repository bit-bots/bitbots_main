import time

import math

import rospy
from humanoid_league_msgs.msg import HeadMode, BallInImage, BallsInImage
import rosparam
from sensor_msgs.msg import JointState, CameraInfo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadCapsule:
    def __init__(self):

        self.config = rosparam.get_param("Behaviour/Head")
        self.delta = self.config["Search"]["headTurnPrecision"]
        self.wait_time = self.config["Search"]["headTurnTime"]
        self.pan_speed_max = self.config["Search"]["maxPanSpeedSearch"]
        self.tilt_speed_max = self.config["Search"]["maxTiltSpeedSearch"]
        self.max_pan = self.config["Camera"]["maxPan"]
        self.min_pan = self.config["Camera"]["minPan"]
        self.max_tilt = self.config["Camera"]["maxTilt"]
        self.min_tilt = self.config["Camera"]["minTilt"]
        self.camera_height = self.config["Camera"]["cameraHeight"]
        self.ball_height = self.config["Camera"]["ballHeight"]
        self.offset_right = self.config["Search"]["offsetRight"]
        self.offset_down = self.config["Search"]["offsetDown"]
        self.offset_left = self.config["Search"]["offsetLeft"]

        # class variables
        self._headmode = 0
        self.confirmedBall = -999
        self.startedconfirmingball = -999
        self.confirmedGoal = -999
        self.startedconfirminggoal = -999
        self.current_pan_pos = 0
        self.current_tilt_pos = 0
        self.is_ball_tracking_still_active = False
        self.bestball_in_image = None, None
        self.cam_info = 640, 360 #todo aus parametern!

        # preparing message for more performance
        self.pos_msg = JointTrajectory()
        self.pos_msg.joint_names = ["HeadPan", "HeadTilt"]
        self.point_msg = JointTrajectoryPoint()
        self.point_msg.positions = [0, 0]
        self.point_msg.velocities = [0, 0]
        self.pos_msg.points = [self.point_msg]

        self.position_publisher = None  # type: rospy.Publisher

    def send_motor_goals(self, pan_position: float, pan_speed: float, tilt_position: float, tilt_speed: float):
        self.current_pan_pos = pan_position
        self.current_tilt_pos = tilt_position
        posnew = math.radians(pan_position), math.radians(tilt_position)
        velnew = math.radians(pan_speed), math.radians(tilt_speed)
        self.pos_msg.points[0].positions = posnew
        self.pos_msg.points[0].velocities = velnew
        self.pos_msg.header.stamp = rospy.Time.now()
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

    def get_pantilt_from_uv(self, u, v):
        # type: (float, float) -> tuple
        cam_height = self.camera_height
        ball_height = self.ball_height
        if u == 0.0 and v == 0.0:

            pan = self.current_pan_pos
            tilt= self.current_tilt_pos
            return pan, tilt
        elif u == 0 and v > 0:
            pan = -90
        elif u==0 and v<0:
            pan = 90
        else:
            pan = math.degrees(math.atan(v/u))

        tilt = -math.degrees(math.atan((cam_height - ball_height / 2)/(math.sqrt(u ** 2 + v ** 2))))
        return pan, tilt

    def cb_ballinimage(self, balls: BallsInImage):
        if len(balls.candidates):
            ball = balls.candidates[0]
            self.bestball_in_image = ball.center.x, ball.center.y

    def get_started_confirm_ball(self):
        return self.startedconfirmingball

    def set_started_confirm_ball(self, t=None):
        if not t:
            t = rospy.get_time()
        self.startedconfirmingball = t

    def unset_started_confirm_ball(self):
        self.startedconfirmingball = -999

    def set_confirmed_ball(self):
        self.startedconfirmingball = rospy.get_time()
        
    def get_started_confirm_goal(self):
        return self.startedconfirminggoal

    def set_started_confirm_goal(self, t=None):
        if not t:
            t = rospy.get_time()
        self.startedconfirminggoal = t

    def unset_started_confirm_goal(self):
        self.startedconfirminggoal = -999

    def set_confirmed_goal(self):
        self.startedconfirminggoal = rospy.get_time()

    def cb_motor_postions(self, msg: JointTrajectory):
        self.pos_msg = msg

    def cb_cam_info(self, msg: CameraInfo):
        self.cam_info = msg.width, msg.height
