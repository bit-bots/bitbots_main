from typing import Optional

import numpy
from geometry_msgs.msg import Twist
from livelybot_msg.msg import PowerSwitch
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty as EmptyMsg

from bitbots_hcm.type_utils import T_RobotControlState
from bitbots_msgs.action import PlayAnimation
from bitbots_msgs.msg import TTS, JointTorque, RobotControlState
from bitbots_msgs.srv import SetTeachingMode


class HcmBlackboard:
    def __init__(self, node: Node):
        self.node = node

        # Basic state
        self.current_state: T_RobotControlState = RobotControlState.STARTUP
        self.stopped: bool = False
        self.game_controller_stop: bool = False

        # Save start time
        self.start_time: Time = self.node.get_clock().now()

        # Get parameters
        self.simulation_active: bool = self.node.get_parameter("simulation_active").value
        self.visualization_active: bool = self.node.get_parameter("visualization_active").value

        # Create service clients
        self.motor_switch_pub = self.node.create_publisher(PowerSwitch, "/power_switch_control", 10)

        # Create action clients and corresponding goal handles
        self.animation_action_client: ActionClient = ActionClient(self.node, PlayAnimation, "animation")
        self.animation_action_current_goal: Optional[Future] = None

        # Create publishers
        self.walk_pub = self.node.create_publisher(Twist, "cmd_vel", 1)
        self.cancel_path_planning_pub = self.node.create_publisher(EmptyMsg, "pathfinding/cancel", 1)
        self.speak_publisher = self.node.create_publisher(TTS, "speak", 1)
        self.torque_publisher = self.node.create_publisher(JointTorque, "set_torque_individual", 10)

        # Latest imu data
        self.accel = numpy.array([0, 0, 0])
        self.gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_much_smoothed_gyro = numpy.array([0, 0, 0])
        self.quaternion = numpy.array([0, 0, 0, 1.0])

        # Animation
        # Animation states
        self.external_animation_running: bool = False
        self.last_animation_goal_time: Optional[Time] = None
        self.last_animation_start_time: Optional[Time] = None
        self.record_active: bool = False

        # Get animation parameters
        self.animation_name_falling_back: str = self.node.get_parameter("animations.falling_back").value
        self.animation_name_falling_front: str = self.node.get_parameter("animations.falling_front").value
        self.animation_name_falling_left: str = self.node.get_parameter("animations.falling_left").value
        self.animation_name_falling_right: str = self.node.get_parameter("animations.falling_right").value
        self.animation_name_falling: str = self.node.get_parameter("animations.falling").value
        self.animation_name_init: str = self.node.get_parameter("animations.init").value
        self.animation_name_stand_up_back: str = self.node.get_parameter("animations.stand_up_back").value
        self.animation_name_stand_up_front: str = self.node.get_parameter("animations.stand_up_front").value
        self.animation_name_startup: str = self.node.get_parameter("animations.startup").value
        self.animation_name_walk_ready: str = self.node.get_parameter("animations.walk_ready").value
        self.animation_name_turning_front_left: str = self.node.get_parameter("animations.turning_front_left").value
        self.animation_name_turning_front_right: str = self.node.get_parameter("animations.turning_front_right").value

        # Teaching State
        self.teaching_mode_state: int = SetTeachingMode.Request.OFF

        # Motor State
        self.current_joint_state: Optional[JointState] = None
        self.previous_joint_state: Optional[JointState] = None
        self.last_different_joint_state_time: Optional[Time] = None
        self.is_power_on: bool = True  # TODO: This never gets updated, but read in check_hardware decision

        # Motor Parameters
        self.motor_timeout_duration: float = self.node.get_parameter("motor_timeout_duration").value
        self.motor_off_time: float = self.node.get_parameter("motor_off_time").value
        self.imu_timeout_duration: float = self.node.get_parameter("imu_timeout_duration").value

        # Walking state
        self.last_walking_goal_time: Optional[Time] = None
        # Time of the last walk motor goal that deviated significantly
        self.last_significant_walk_motion_time: Optional[Time] = None
        # Seconds without significant walk motion before dropping from WALKING to CONTROLLABLE
        self.standing_transition_delay: float = self.node.get_parameter("standing_transition_delay").value

        # Falling
        # Parameters
        self.is_stand_up_active = self.node.get_parameter("stand_up_active").value
        self.falling_detection_active = self.node.get_parameter("falling_active").value
        self.in_squat: bool = False  # Needed for sequencing of the stand up motion

        # Kicking
        # State
        self.last_kick_goal_time: Optional[Time] = None

        # IMU state
        self.imu_msg: Optional[Imu] = None
        self.previous_imu_msg: Optional[Imu] = None
        self.last_different_imu_state_time: Optional[Time] = None

        # Battery state
        self.battery_voltage_threshold: float = self.node.get_parameter("battery.voltage_threshold").value
        self.battery_debounce_time: float = self.node.get_parameter("battery.debounce_time").value
        self.battery_voltage: Optional[float] = None

        # Diagnostics state
        self.servo_diag_error: bool = False
        self.servo_overload: bool = False
        self.servo_overheat: bool = False
        self.imu_diag_error: bool = False
        self.pressure_diag_error: bool = False

    def cancel_path_planning(self):
        self.cancel_path_planning_pub.publish(EmptyMsg())
