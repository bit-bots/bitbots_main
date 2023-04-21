import json
import os
from typing import List, Optional

import numpy
from ament_index_python import get_package_share_directory
from bitbots_hcm.fall_checker import FallChecker
from bitbots_hcm.fall_classifier import FallClassifier
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PointStamped, Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import SetBool

from bitbots_msgs.action import Dynup
from bitbots_msgs.msg import JointCommand
from humanoid_league_msgs.action import PlayAnimation
from humanoid_league_msgs.msg import RobotControlState


class HcmBlackboard():
    def __init__(self, node: Node):
        self.node = node
        self.current_state: RobotControlState = RobotControlState.STARTUP
        self.stopped: bool = False
        self.shut_down_request: bool  = False
        self.simulation_active = self.node.get_parameter('simulation_active').get_parameter_value().bool_value
        self.visualization_active = self.node.get_parameter('visualization_active').get_parameter_value().bool_value

        # this is used to prevent calling Time a lot, which takes some time
        # we assume that the time does not change during one update cycle
        self.current_time = self.node.get_clock().now()
        self.start_time = self.current_time
        # Imu
        self.accel = numpy.array([0, 0, 0])
        self.gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_much_smoothed_gyro = numpy.array([0, 0, 0])
        self.quaternion = numpy.array([0, 0, 0, 0.21])
        self.pickup_accel_threshold: float = 1000.0

        # Pressure sensors
        self.pressure_sensors_installed = self.node.get_parameter('pressure_sensors_installed').get_parameter_value().bool_value
        # initialize values high to prevent wrongly thinking the robot is picked up during start or in simulation
        self.pressures: List[float] = [100.0] * 8
        foot_zero_service_name = self.node.get_parameter("foot_zero_service").get_parameter_value().string_value
        self.foot_zero_service = self.node.create_client(EmptySrv, foot_zero_service_name)

        self.motor_switch_service = self.node.create_client(SetBool, 'core/switch_power')

        # Animation
        self.animation_action_client: Optional[ActionClient] = None
        self.animation_action_current_goal: Optional[PlayAnimation.Goal]  = None
        self.dynup_action_client: Optional[ActionClient] = None
        self.dynup_action_current_goal: Optional[Dynup.Goal] = None
        self.last_animation_goal_time = self.node.get_clock().now()
        self.external_animation_running: bool = False
        self.animation_requested: bool = False
        if self.simulation_active:
            self.walkready_animation = self.node.get_parameter("animations.walkready_sim").get_parameter_value().string_value
        else:
            self.walkready_animation = self.node.get_parameter("animations.walkready").get_parameter_value().string_value
        self.falling_animation_front = self.node.get_parameter("animations.falling_front").get_parameter_value().string_value
        self.falling_animation_back = self.node.get_parameter("animations.falling_back").get_parameter_value().string_value
        self.falling_animation_left = self.node.get_parameter("animations.falling_left").get_parameter_value().string_value
        self.falling_animation_right = self.node.get_parameter("animations.falling_right").get_parameter_value().string_value
        self.stop_animation = self.node.get_parameter("animations.penalty").get_parameter_value().string_value
        self.sit_down_animation = self.node.get_parameter("animations.sit_down").get_parameter_value().string_value
        self.motor_off_animation = self.node.get_parameter("animations.motor_off").get_parameter_value().string_value
        self.stand_up_front_animation = self.node.get_parameter("animations.stand_up_front").get_parameter_value().string_value
        self.stand_up_back_animation = self.node.get_parameter("animations.stand_up_back").get_parameter_value().string_value
        self.stand_up_left_animation = self.node.get_parameter("animations.stand_up_left").get_parameter_value().string_value
        self.stand_up_right_animation = self.node.get_parameter("animations.stand_up_right").get_parameter_value().string_value
        # motors
        # initialize with current time, or motors will be turned off on start
        self.last_motor_goal_time = self.node.get_clock().now()
        self.last_motor_update_time =  Time(seconds=int(0), nanoseconds=0)
        self.motor_timeout_duration = self.node.get_parameter("motor_timeout_duration").get_parameter_value().double_value
        self.motor_off_time = self.node.get_parameter("motor_off_time").get_parameter_value().double_value
        self.current_joint_state: Optional[JointState] = None
        self.previous_joint_state: Optional[JointState] = None
        anim_package = self.node.get_parameter("animations.anim_package").get_parameter_value().string_value
        path = get_package_share_directory(anim_package)
        path = os.path.join(path, 'animations/motion/', self.walkready_animation + '.json')
        with open(path, 'r') as f:
            json_data = json.load(f)
        keyframes: List[dict] = json_data["keyframes"]
        self.walkready_pose_dict: dict = keyframes[-1]["goals"]
        self.walkready_pose_threshold = self.node.get_parameter("animations.walkready_pose_threshold").get_parameter_value().double_value
        self.is_power_on: bool = False
        # walking
        self.last_walking_goal_time = self.node.get_clock().now()
        self.walk_pub = self.node.create_publisher(Twist, "cmd_vel", 1)

        self.record_active: bool = False

        # falling
        self.fall_checker = FallChecker(self.node)
        self.is_stand_up_active = self.node.get_parameter('stand_up_active').get_parameter_value().bool_value
        self.falling_detection_active = self.node.get_parameter('falling_active').get_parameter_value().bool_value
        self.joint_pub = self.node.create_publisher(JointCommand, "DynamixelController/command", 1)

        # kicking
        self.last_kick_feedback: Optional[Time] = None

        # direct messages for falling classier
        # todo needs refactoring
        path = get_package_share_directory('bitbots_hcm')
        smooth_threshold = self.node.get_parameter('smooth_threshold').get_parameter_value().double_value
        self.classifier = FallClassifier(path + "/classifier/", smooth_threshold=smooth_threshold)
        self.imu_msg: Optional[Imu] = None
        self.cop_l_msg: Optional[PointStamped] = None
        self.cop_r_msg: Optional[PointStamped] = None

        self.servo_diag_error: bool = False
        self.servo_overload: bool = False
        self.imu_diag_error: bool = False
        self.pressure_diag_error: bool = False

        self.move_base_cancel_pub = self.node.create_publisher(EmptyMsg, "pathfinding/cancel", 1)

    def diag_cb(self, msg: DiagnosticArray):
        status: DiagnosticStatus
        for status in msg.status:
            if "//Servos/" in status.name:
                if status.level == DiagnosticStatus.ERROR and "Overload" in status.message:
                    self.servo_overload = True
            elif "//Servos" in status.name:
                self.servo_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE
            elif "//IMU" in status.name:
                self.imu_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE
            elif "//Pressure" in status.name:
                self.pressure_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE

    def last_kick_feedback_callback(self, msg: PlayAnimation.Feedback):
        self.last_kick_feedback = self.node.get_clock().now()

    def cancel_move_base_goal(self):
        self.move_base_cancel_pub.publish(EmptyMsg())
