import json
import os
from typing import List, Optional

import numpy
from ament_index_python import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PointStamped, Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.task import Future
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

        # Basic state
        self.current_state: RobotControlState = RobotControlState.STARTUP
        self.stopped: bool = False
        self.shut_down_request: bool  = False

        # Save start time
        self.start_time: Time = self.node.get_clock().now()

        # Get parameters
        self.simulation_active: bool = self.node.get_parameter('simulation_active').value
        self.visualization_active: bool = self.node.get_parameter('visualization_active').value
        self.pickup_accel_threshold: float = 1000.0  # TODO: make this a parameter
        self.pressure_sensors_installed: bool = self.node.get_parameter('pressure_sensors_installed').value

        # Create services
        foot_zero_service_name = self.node.get_parameter("foot_zero_service").value
        self.foot_zero_service = self.node.create_client(EmptySrv, foot_zero_service_name)
        self.motor_switch_service = self.node.create_client(SetBool, 'core/switch_power')

        # Create action clients and corresponding goal handles
        self.animation_action_client: ActionClient = ActionClient(self.node, PlayAnimation, 'animation')
        self.animation_action_current_goal: Optional[Future]  = None
        self.dynup_action_client: ActionClient = ActionClient(self.node, Dynup, 'dynup')
        self.dynup_action_current_goal: Optional[Future] = None

        # Create publishers
        self.walk_pub = self.node.create_publisher(Twist, "cmd_vel", 1)
        self.cancel_path_planning_pub = self.node.create_publisher(EmptyMsg, "pathfinding/cancel", 1)

        # Latest imu data
        self.accel = numpy.array([0, 0, 0])
        self.gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_much_smoothed_gyro = numpy.array([0, 0, 0])
        self.quaternion = numpy.array([0, 0, 0, 1.0])

        # Pressure sensors
        # Initialize values high to prevent wrongly thinking the robot is picked up during start or in simulation
        self.pressures: List[float] = [100.0] * 8

        # Animation
        # Animation states
        self.animation_requested: bool = False
        self.external_animation_running: bool = False
        self.last_animation_goal_time: Time = self.node.get_clock().now()
        self.record_active: bool = False

        if self.simulation_active:
            self.walkready_animation = self.node.get_parameter("animations.walkready_sim").value
        else:
            self.walkready_animation = self.node.get_parameter("animations.walkready").value

        # Get animation parameters
        self.falling_animation_front = self.node.get_parameter("animations.falling_front").value
        self.falling_animation_back = self.node.get_parameter("animations.falling_back").value
        self.falling_animation_left = self.node.get_parameter("animations.falling_left").value
        self.falling_animation_right = self.node.get_parameter("animations.falling_right").value

        # Motors

        # State
        # Initialize with current time, or motors will be turned off on start
        self.last_motor_goal_time: Time = self.node.get_clock().now()
        self.last_motor_update_time =  Time()
        self.current_joint_state: Optional[JointState] = None
        self.previous_joint_state: Optional[JointState] = None
        self.is_power_on: bool = False

        # Parameters
        self.motor_timeout_duration: float = self.node.get_parameter("motor_timeout_duration").value
        self.motor_off_time: float = self.node.get_parameter("motor_off_time").value
        self.imu_timeout_duration: float = self.node.get_parameter("imu_timeout_duration").value
        self.walkready_pose_threshold = self.node.get_parameter("animations.walkready_pose_threshold").value


        # Hacky way of getting the walkready pose. TODO change that. Maybe use a dynup service
        anim_package = self.node.get_parameter("animations.anim_package").value
        path = get_package_share_directory(anim_package)
        path = os.path.join(path, 'animations/motion/', self.walkready_animation + '.json')
        with open(path, 'r') as f:
            json_data = json.load(f)
        keyframes: List[dict] = json_data["keyframes"]
        self.walkready_pose_dict: dict = keyframes[-1]["goals"]

        # Walking state
        self.last_walking_goal_time = self.node.get_clock().now()

        # Falling
        # Paramerters
        self.is_stand_up_active = self.node.get_parameter('stand_up_active').value
        self.falling_detection_active = self.node.get_parameter('falling_active').value

        # Kicking
        # State
        self.last_kick_feedback: Optional[Time] = None

        # IMU state
        self.imu_msg: Optional[Imu] = None

        # Diagnostics state
        self.servo_diag_error: bool = False
        self.servo_overload: bool = False
        self.imu_diag_error: bool = False
        self.pressure_diag_error: bool = False

    def diag_cb(self, msg: DiagnosticArray):
        status: DiagnosticStatus
        for status in msg.status:
            if "//Servos/" in status.name:
                if status.level == DiagnosticStatus.ERROR and "Overload" in status.message:
                    self.servo_overload = True
            elif "//Servos" in status.name:
                self.servo_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)
            elif "//IMU" in status.name:
                self.imu_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)
            elif "//Pressure" in status.name:
                self.pressure_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)

    def last_kick_feedback_callback(self, msg: PlayAnimation.Feedback):
        self.last_kick_feedback = self.node.get_clock().now()

    def cancel_path_planning(self):
        self.cancel_path_planning_pub.publish(EmptyMsg())
