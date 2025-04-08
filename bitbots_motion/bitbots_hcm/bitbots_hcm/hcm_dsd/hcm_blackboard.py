from typing import Optional

import numpy
from bitbots_utils.utils import get_parameters_from_other_node_sync
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import SetBool

from bitbots_hcm.type_utils import T_RobotControlState
from bitbots_msgs.action import Dynup, PlayAnimation
from bitbots_msgs.msg import Audio, JointTorque, RobotControlState
from bitbots_msgs.srv import SetTeachingMode


class HcmBlackboard:
    def __init__(self, node: Node):
        self.node = node

        # Basic state
        self.current_state: T_RobotControlState = RobotControlState.STARTUP
        self.stopped: bool = False

        # Save start time
        self.start_time: Time = self.node.get_clock().now()

        # Get parameters
        self.simulation_active: bool = self.node.get_parameter("simulation_active").value
        self.visualization_active: bool = self.node.get_parameter("visualization_active").value
        self.pickup_accel_threshold: float = self.node.get_parameter("pick_up_accel_threshold").value
        self.pressure_sensors_installed: bool = self.node.get_parameter("pressure_sensors_installed").value
        self.motor_start_delay: int = 0
        if not self.simulation_active:  # The hardware interface is obviously not available in simulation
            self.motor_start_delay = get_parameters_from_other_node_sync(
                self.node, "/wolfgang_hardware_interface", ["start_delay"]
            )["start_delay"]

        # Create service clients
        self.foot_zero_service = self.node.create_client(EmptySrv, "set_foot_zero")
        self.motor_switch_service = self.node.create_client(SetBool, "core/switch_power")

        # Create action clients and corresponding goal handles
        self.animation_action_client: ActionClient = ActionClient(self.node, PlayAnimation, "animation")
        self.animation_action_current_goal: Optional[Future] = None
        self.dynup_action_client: ActionClient = ActionClient(self.node, Dynup, "dynup")
        self.dynup_action_current_goal: Optional[Future] = None

        # Create publishers
        self.walk_pub = self.node.create_publisher(Twist, "cmd_vel", 1)
        self.cancel_path_planning_pub = self.node.create_publisher(EmptyMsg, "pathfinding/cancel", 1)
        self.speak_publisher = self.node.create_publisher(Audio, "speak", 1)
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
        self.animation_name_init: str = self.node.get_parameter("animations.init").value
        self.animation_name_stand_up_back: str = self.node.get_parameter("animations.stand_up_back").value
        self.animation_name_stand_up_front: str = self.node.get_parameter("animations.stand_up_front").value
        self.animation_name_startup: str = self.node.get_parameter("animations.startup").value
        self.animation_name_turning_front_left: str = self.node.get_parameter("animations.turning_front_left").value
        self.animation_name_turning_front_right: str = self.node.get_parameter("animations.turning_front_right").value

        # Teaching State
        self.teaching_mode_state: int = SetTeachingMode.Request.OFF

        # Motor State
        self.current_joint_state: Optional[JointState] = None
        self.previous_joint_state: Optional[JointState] = None
        self.last_different_joint_state_time: Optional[Time] = None
        self.is_power_on: bool = False

        # Motor Parameters
        self.motor_timeout_duration: float = self.node.get_parameter("motor_timeout_duration").value
        self.motor_off_time: float = self.node.get_parameter("motor_off_time").value
        self.imu_timeout_duration: float = self.node.get_parameter("imu_timeout_duration").value

        # Walking state
        self.last_walking_goal_time: Optional[Time] = None

        # Falling
        # Paramerters
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

        # Pressure sensors
        # Initialize values high to prevent wrongly thinking the robot is picked up during start or in simulation
        self.pressures: list[float] = [100.0] * 8
        self.previous_pressures: list[float] = self.pressures.copy()
        self.last_different_pressure_state_time: Optional[Time] = None

        # Diagnostics state
        self.servo_diag_error: bool = False
        self.servo_overload: bool = False
        self.servo_overheat: bool = False
        self.imu_diag_error: bool = False
        self.pressure_diag_error: bool = False

    def cancel_path_planning(self):
        self.cancel_path_planning_pub.publish(EmptyMsg())
