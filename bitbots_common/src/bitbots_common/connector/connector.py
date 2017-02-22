#!/usr/bin/env python3

import time

from model.capsules.animation_capsule import AnimationCapsule
from model.capsules.blackboard_capsule import BlackboardCapsule
from model.capsules.game_status_capsule import GameStatusCapsule
from model.capsules.team_data_capsule import TeamDataCapsule
from model.capsules.walking_capsule import WalkingCapsule
from model.capsules.world_model_capsule import WorldModelCapsule

import rospy
from bitbots_misc.bitbots_common.src.bitbots_common.connector.capsules.head_capsule import HeadCapsule
from bitbots_misc.bitbots_common.src.bitbots_common.connector.capsules.vision_capsule import VisionCapsule
from humanoid_league_msgs.msg import Role, BallRelative
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Connector:
    def __init__(self):
        self.vision = VisionCapsule()
        self.world_model = WorldModelCapsule()
        self.blackboard = BlackboardCapsule()
        self.gamestate = GameStatusCapsule()
        self.walking = WalkingCapsule()
        self.team_data = TeamDataCapsule()
        self.animation = AnimationCapsule()
        self.head = HeadCapsule()

        self.head_pub = None  # type: rospy.Publisher
        self.speaker = None  # type: rospy.Publisher
        self.config = None  # type: dict

        self.delta = self.config["Search"]["headTurnPrecision"]
        self.wait_time = self.config["Search"]["headTurnTime"]
        self.pan_speed_max = self.config["Search"]["maxPanSpeedSearch"]
        self.tilt_speed_max = self.config["Search"]["maxTiltSpeedSearch"]

        # class variables
        self.current_pan_pos = 0
        self.current_tilt_pos = 0

      #  self.ball = BallRelative()
        self.my_data = dict()

        self.role = 0

        self.is_ball_tracking_still_active = False

        # preparing message for more performance
        self.pos_msg = JointTrajectory()
        self.pos_msg.joint_names = ["HeadPan", "HeadTilt"]
        self.point_msg = JointTrajectoryPoint()
        self.pos_msg.points = [self.point_msg]

        self.position_publisher = rospy.Publisher("/head_motor_goals", JointTrajectory, queue_size=10)
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        rospy.Subscriber("/role", Role, self.role_cb)
        rospy.Subscriber("/ball_relative", BallRelative, self.ball_callback)

    def send_motor_goals(self, pan_position: float, pan_speed: float, tilt_position: float, tilt_speed: float):
        self.point_msg.positions = [pan_position, tilt_position]
        self.point_msg.velocities = [pan_speed, tilt_speed]
        self.position_publisher.publish(self.pos_msg)

    def role_cb(self, msg):
        self.role = msg.role

    def ball_callback(self, ball: BallRelative):
        self.ball = ball
        self.my_data["BallLastSeen"] = time.time()

    def get_current_head_pos(self):
        return self.current_pan_pos, self.current_tilt_pos

    def get_duty(self):
        return self.role
