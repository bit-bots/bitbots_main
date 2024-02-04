#!/usr/bin/env python3

import os
import threading

import rclpy
from ament_index_python import get_package_share_directory
from bitbots_tts.tts import speak
from bitbots_utils.utils import get_parameters_from_ros_yaml
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from dynamic_stack_decider.dsd import DSD
from rcl_interfaces.msg import Parameter as ParameterMsg
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from ros2_numpy import numpify
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool

from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard
from bitbots_msgs.msg import FootPressure, RobotControlState


class HardwareControlManager:
    def __init__(self, use_sim_time, simulation_active, visualization_active):
        rclpy.init(args=None)
        node_name = "hcm_py"

        # Load parameters from yaml file because this is a hacky cpp python hybrid node for performance reasons
        parameter_msgs: list(ParameterMsg) = get_parameters_from_ros_yaml(
            node_name, f"{get_package_share_directory('bitbots_hcm')}/config/hcm_wolfgang.yaml", use_wildcard=True
        )
        parameters = []
        for parameter_msg in parameter_msgs:
            parameters.append(Parameter.from_parameter_msg(parameter_msg))
        if use_sim_time:
            parameters.append(Parameter("use_sim_time", type_=Parameter.Type.BOOL, value=True))
        if simulation_active:
            parameters.append(Parameter("simulation_active", type_=Parameter.Type.BOOL, value=True))
        if visualization_active:
            parameters.append(Parameter("visualization_active", type_=Parameter.Type.BOOL, value=True))

        # Create Python node
        self.node = Node(
            node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=parameters,
        )

        # Create own executor for Python part
        multi_executor = MultiThreadedExecutor()
        multi_executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=multi_executor.spin, args=(), daemon=True)
        self.spin_thread.start()

        # Otherwise messages will get lost, bc the init is not finished
        self.node.get_clock().sleep_for(Duration(seconds=0.5))
        self.node.get_logger().debug("Starting hcm")

        # Create Dynamic Stack Decider Blackboard
        self.blackboard = HcmBlackboard(self.node)
        # Create Dynamic Stack Decider
        self.dsd = DSD(self.blackboard, "debug/dsd/hcm", node=self.node)
        # Get the path to the python actions and decisions
        dirname = os.path.join(get_package_share_directory("bitbots_hcm"), "hcm_dsd")
        # Register actions and decisions
        self.dsd.register_actions(os.path.join(dirname, "actions"))
        self.dsd.register_decisions(os.path.join(dirname, "decisions"))
        # Load the behavior file
        self.dsd.load_behavior(os.path.join(dirname, "hcm.dsd"))

        # Flag to deactivate the HCM
        self.hcm_deactivated = False

        # Create subscribers
        self.node.create_subscription(Bool, "pause", self.pause, 1)
        self.node.create_subscription(Bool, "core/power_switch_status", self.power_cb, 1)
        self.node.create_subscription(Bool, "hcm_deactivate", self.deactivate_cb, 1)
        self.node.create_subscription(DiagnosticArray, "diagnostics_agg", self.diag_cb, 1)

        # Store time of the last tick
        self.last_tick_start_time = self.node.get_clock().now()

        # Anounce the HCM startup
        speak("Starting HCM", self.blackboard.speak_publisher, priority=50)

    def tick(self):
        """
        Keeps updating the DSD and publishes its current state.
        All the forwarding of joint goals is directly done in the callbacks to reduce latency.
        """
        # Store the time of the current tick
        tick_start_time = self.node.get_clock().now()
        # This can happen in simulation due to bad implementation in rclpy
        if self.last_tick_start_time != tick_start_time:
            self.last_tick_start_time = tick_start_time
            # Do not perform any behavior if the HCM is deactivated
            if self.hcm_deactivated:
                self.blackboard.current_state = RobotControlState.CONTROLLABLE
            else:
                try:
                    # Step the dsd
                    self.dsd.update()
                except IndexError:
                    # this error will happen during shutdown procedure, just ignore it
                    pass

    def deactivate_cb(self, msg: Bool):
        """Deactivates the HCM."""
        self.hcm_deactivated = msg.data

    def pause(self, msg: Bool):
        """Updates the stop state for the state machine"""
        self.blackboard.stopped = msg.data

    def power_cb(self, msg: Bool):
        """Updates the power state."""
        self.blackboard.is_power_on = msg.data

    def diag_cb(self, msg: DiagnosticArray):
        """Updates the diagnostic state."""
        status: DiagnosticStatus
        for status in msg.status:
            if "//Servos/" in status.name:
                if status.level == DiagnosticStatus.ERROR and "Overload" in status.message:
                    self.blackboard.servo_overload = True
            elif "//Servos" in status.name:
                self.blackboard.servo_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)
            elif "//IMU" in status.name:
                self.blackboard.imu_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)
            elif "//Pressure" in status.name:
                self.blackboard.pressure_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)

    def get_state(self) -> RobotControlState:
        """Returns the current state of the HCM."""
        return self.blackboard.current_state

    # The following methods are used to set the blackboard values from the cpp part

    def set_animation_requested(self, animation_requested: bool):
        self.blackboard.animation_requested = animation_requested

    def set_external_animation_running(self, running: bool):
        self.blackboard.external_animation_running = running

    def set_record_active(self, active: bool):
        self.blackboard.record_active = active

    def set_last_animation_goal_time(self, time_msg_serialized: bytes):
        self.blackboard.last_animation_goal_time = deserialize_message(time_msg_serialized, TimeMsg)

    def set_last_walking_goal_time(self, time_msg_serialized: bytes):
        self.blackboard.last_walking_goal_time = Time.from_msg(deserialize_message(time_msg_serialized, TimeMsg))

    def set_last_kick_goal_time(self, time_msg_serialized: bytes):
        self.blackboard.last_kick_goal_time = Time.from_msg(deserialize_message(time_msg_serialized, TimeMsg))

    def set_current_joint_state(self, joint_state_msg_serialized: bytes):
        self.blackboard.previous_joint_state = self.blackboard.current_joint_state
        self.blackboard.current_joint_state = deserialize_message(joint_state_msg_serialized, JointState)

    def set_pressure_left(self, pressure_msg_serialized: bytes):
        msg: FootPressure = deserialize_message(pressure_msg_serialized, FootPressure)
        self.blackboard.previous_pressures = self.blackboard.pressures
        self.blackboard.pressures[0] = msg.left_front
        self.blackboard.pressures[1] = msg.left_back
        self.blackboard.pressures[2] = msg.right_front
        self.blackboard.pressures[3] = msg.right_back

    def set_pressure_right(self, pressure_msg_serialized: bytes):
        msg: FootPressure = deserialize_message(pressure_msg_serialized, FootPressure)
        self.blackboard.previous_pressures = self.blackboard.pressures
        self.blackboard.pressures[4] = msg.left_front
        self.blackboard.pressures[5] = msg.left_back
        self.blackboard.pressures[6] = msg.right_front
        self.blackboard.pressures[7] = msg.right_back

    def set_imu(self, imu_msg_serialized: bytes):
        self.blackboard.previous_imu_msg = self.blackboard.imu_msg

        msg: Imu = deserialize_message(imu_msg_serialized, Imu)

        self.blackboard.accel = numpify(msg.linear_acceleration)
        self.blackboard.gyro = numpify(msg.angular_velocity)
        self.blackboard.quaternion = numpify(msg.orientation)

        self.blackboard.smooth_gyro = 0.95 * self.blackboard.smooth_gyro + 0.05 * self.blackboard.gyro
        self.blackboard.smooth_accel = 0.99 * self.blackboard.smooth_accel + 0.01 * self.blackboard.accel
        self.blackboard.not_much_smoothed_gyro = (
            0.5 * self.blackboard.not_much_smoothed_gyro + 0.5 * self.blackboard.gyro
        )

        self.blackboard.imu_msg = msg
