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
from std_srvs.srv import SetBool

from bitbots_hcm import hcm_dsd
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard
from bitbots_hcm.type_utils import T_RobotControlState
from bitbots_msgs.msg import FootPressure, RobotControlState
from bitbots_msgs.srv import ManualPenalize, SetTeachingMode


class HardwareControlManager:
    def __init__(self, use_sim_time, simulation_active, visualization_active):
        rclpy.init(args=None)
        node_name = "hcm_py"

        # Load parameters from yaml file because this is a hacky cpp python hybrid node for performance reasons
        parameter_msgs: list[ParameterMsg] = get_parameters_from_ros_yaml(
            node_name, f"{get_package_share_directory('bitbots_hcm')}/config/hcm_wolfgang.yaml", use_wildcard=True
        )
        parameters = [
            Parameter("use_sim_time", type_=Parameter.Type.BOOL, value=use_sim_time),
            Parameter("simulation_active", type_=Parameter.Type.BOOL, value=simulation_active),
            Parameter("visualization_active", type_=Parameter.Type.BOOL, value=visualization_active),
        ]
        parameters.extend(map(Parameter.from_parameter_msg, parameter_msgs))

        # Create Python node
        self.node = Node(
            node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=parameters,
        )

        # Create own executor for Python part
        multi_executor = MultiThreadedExecutor(num_threads=10)
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
        # Register actions and decisions
        self.dsd.register_actions(hcm_dsd.actions.__path__[0])
        self.dsd.register_decisions(hcm_dsd.decisions.__path__[0])
        # Load the behavior file
        self.dsd.load_behavior(os.path.join(hcm_dsd.__path__[0], "hcm.dsd"))

        # Flag to deactivate the HCM
        self.hcm_deactivated = False

        # Create subscribers
        self.node.create_subscription(Bool, "core/power_switch_status", self.power_cb, 1)
        self.node.create_subscription(Bool, "hcm_deactivate", self.deactivate_cb, 1)
        self.node.create_subscription(DiagnosticArray, "diagnostics_agg", self.diag_cb, 1)

        # Create services
        self.node.create_service(SetBool, "record_mode", self.set_record_mode_callback)
        self.node.create_service(SetBool, "play_animation_mode", self.set_animation_mode_callback)
        self.teaching_mode_service = self.node.create_service(
            SetTeachingMode, "teaching_mode", self.set_teaching_mode_callback
        )
        self.manual_penalize_service = self.node.create_service(
            ManualPenalize, "manual_penalize", self.set_manual_penalize_mode_callback
        )

        # Store time of the last tick
        self.last_tick_start_time = self.node.get_clock().now()

        # Announce the HCM startup
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

    def set_manual_penalize_mode_callback(self, req: ManualPenalize.Request, resp: ManualPenalize.Response):
        """Callback for the manual penalize service."""
        if req.penalize == ManualPenalize.Request.OFF:
            self.blackboard.stopped = False
        elif req.penalize == ManualPenalize.Request.ON:
            self.blackboard.stopped = True
        elif req.penalize == ManualPenalize.Request.SWITCH:
            self.blackboard.stopped = not self.blackboard.stopped
        else:
            self.node.get_logger().error("Manual penalize call with unspecified request")
            resp.success = False
            return resp
        resp.success = True
        return resp

    def power_cb(self, msg: Bool):
        """Updates the power state."""
        self.blackboard.is_power_on = msg.data

    def diag_cb(self, msg: DiagnosticArray):
        """Updates the diagnostic state."""
        status: DiagnosticStatus
        for status in msg.status:
            if "/Servos/" in status.name:
                if status.level == DiagnosticStatus.ERROR:
                    if "Overload" in status.message:
                        self.blackboard.servo_overload = True
                    elif "Overheat" in status.message:
                        self.blackboard.servo_overheat = True

            elif "/Servos" in status.name:
                self.blackboard.servo_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)
            elif "/IMU" in status.name:
                self.blackboard.imu_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)
            elif "/Pressure" in status.name:
                self.blackboard.pressure_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)

    def get_state(self) -> T_RobotControlState:
        """Returns the current state of the HCM."""
        return self.blackboard.current_state

    def set_animation_mode_callback(self, request: SetBool.Request, response: SetBool.Response):
        # Check if the robot is in a state where it is allowed to play animations
        if request.data:  # We want to go into the animation mode
            if self.blackboard.current_state in [RobotControlState.CONTROLLABLE, RobotControlState.RECORD]:
                self.blackboard.last_animation_start_time = self.node.get_clock().now()
                self.blackboard.external_animation_running = True
                response.success = True
                response.message = "Robot is now in animation mode, have fun!"
                return response
            else:
                response.success = False
                response.message = "Robot is not in a state where it is allowed to play new animations"
                return response
        else:  # We want to go out of the animation mode
            if self.blackboard.current_state in [RobotControlState.ANIMATION_RUNNING, RobotControlState.RECORD]:
                self.blackboard.external_animation_running = False
                response.success = True
                response.message = "Animation stopped successfully"
                return response
            else:
                response.success = False
                response.message = "Robot is not in a state where it is allowed to stop animations"
                return response

    def set_record_mode_callback(self, request: SetBool.Request, response: SetBool.Response):
        self.blackboard.record_active = request.data
        response.success = True
        return response

    def set_teaching_mode_callback(
        self, request: SetTeachingMode.Request, response: SetTeachingMode.Response
    ) -> SetTeachingMode.Response:
        # Store modifiable version of the requested state.
        state = request.state

        # Modify requested state if request state is SWITCH so that it switches between HOLD and TEACH after it was turned on once.
        if state == SetTeachingMode.Request.SWITCH:
            if self.blackboard.teaching_mode_state in [SetTeachingMode.Request.HOLD, SetTeachingMode.Request.OFF]:
                state = SetTeachingMode.Request.TEACH
            elif self.blackboard.teaching_mode_state == SetTeachingMode.Request.TEACH:
                state = SetTeachingMode.Request.HOLD

        # Check if we are able to start the teaching mode
        if state == SetTeachingMode.Request.TEACH and self.blackboard.current_state not in [
            RobotControlState.CONTROLLABLE,
            RobotControlState.PICKED_UP,
            RobotControlState.PENALTY,
            RobotControlState.RECORD,
        ]:
            # Respond that we can not activate the teaching mode in the current state
            response.success = False
            return response

        if (
            state == SetTeachingMode.Request.HOLD
            and self.blackboard.teaching_mode_state != SetTeachingMode.Request.TEACH
        ):
            response.success = False
            return response

        # Activate / Deactivate teaching mode
        self.blackboard.teaching_mode_state = state
        response.success = True
        return response

    # The following methods are used to set the blackboard values from the cpp part

    def set_last_animation_goal_time(self, time_msg_serialized: bytes):
        self.blackboard.last_animation_goal_time = Time.from_msg(deserialize_message(time_msg_serialized, TimeMsg))

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
