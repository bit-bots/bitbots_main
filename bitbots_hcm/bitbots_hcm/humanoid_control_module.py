#!/usr/bin/env python3
from platform import node
from sqlite3 import Time
import numpy
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from geometry_msgs.msg import PointStamped

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Imu, JointState
from diagnostic_msgs.msg import DiagnosticArray

from humanoid_league_msgs.msg import Animation as AnimationMsg, RobotControlState, Audio
from humanoid_league_msgs.action import PlayAnimation
from humanoid_league_speaker.speaker import speak
from bitbots_msgs.msg import FootPressure
from bitbots_msgs.action import Dynup, Kick

from bitbots_msgs.msg import JointCommand
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard
from dynamic_stack_decider.dsd import DSD
import os
import threading
from rclpy.executors import ExternalShutdownException
from bitbots_utils.utils import get_parameters_from_ros_yaml
from ament_index_python import get_package_share_directory
from rcl_interfaces.msg import Parameter as ParameterMsg
from rclpy.serialization import serialize_message, deserialize_message


class HardwareControlManager:

    def __init__(self, use_sim_time, simulation_active, visualization_active):
        rclpy.init(args=None)
        node_name = "hcm_py"
        parameter_msgs: list(ParameterMsg) = get_parameters_from_ros_yaml(
            node_name, f"{get_package_share_directory('bitbots_hcm')}/config/hcm_wolfgang.yaml", use_wildcard=True)
        parameters = []
        for parameter_msg in parameter_msgs:
            parameters.append(Parameter.from_parameter_msg(parameter_msg))
        if use_sim_time:
            parameters.append(Parameter("use_sime_time", type_=Parameter.Type.BOOL, value=True))
        if simulation_active:
            parameters.append(Parameter("simulation_active", type_=Parameter.Type.BOOL, value=True))
        if visualization_active:
            parameters.append(Parameter("visualization_active", type_=Parameter.Type.BOOL, value=True))
        self.node = Node(node_name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True,
                         parameter_overrides=parameters)
        # create own executor for python part
        multi_executor = MultiThreadedExecutor()
        multi_executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=multi_executor.spin, args=(), daemon=True)
        #self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

        self.blackboard = None

        # --- Initialize Node ---
        # Otherwise messages will get lost, bc the init is not finished
        self.node.get_clock().sleep_for(Duration(seconds=0.1))
        self.node.get_logger().debug("Starting hcm")
        self.simulation_active = self.node.get_parameter("simulation_active")
        # dsd
        self.blackboard = HcmBlackboard(self.node)
        self.blackboard.animation_action_client = ActionClient(self.node, PlayAnimation, 'animation')
        self.blackboard.dynup_action_client = ActionClient(self.node, Dynup, 'dynup')
        dirname = os.path.dirname(os.path.realpath(__file__)) + "/hcm_dsd"
        self.dsd = DSD(self.blackboard, "debug/dsd/hcm", node=self.node)
        self.dsd.register_actions(os.path.join(dirname, 'actions'))
        self.dsd.register_decisions(os.path.join(dirname, 'decisions'))
        self.dsd.load_behavior(os.path.join(dirname, 'hcm.dsd'))
        self.hcm_deactivated = False
        # Publisher / subscriber
        self.blackboard.speak_publisher = self.node.create_publisher(Audio, 'speak', 1)

        # important to make sure the connection to the speaker is established, for next line
        self.node.get_clock().sleep_for(Duration(seconds=0.1))
        speak("Starting hcm", self.blackboard.speak_publisher, priority=50)
        self.node.create_subscription(Bool, "pause", self.pause, 1)
        self.node.create_subscription(Bool, "core/power_switch_status", self.power_cb, 1)
        self.node.create_subscription(Bool, "hcm_deactivate", self.deactivate_cb, 1)
        self.node.create_subscription(DiagnosticArray, "diagnostics_agg", self.blackboard.diag_cb, 1)
        self.node.add_on_set_parameters_callback(self.on_set_parameters)

        self.last_loop_start_time = self.node.get_clock().now()

    def deactivate_cb(self, msg):
        self.hcm_deactivated = msg.data

    def pause(self, msg):
        """ Updates the stop state for the state machine"""
        self.blackboard.stopped = msg.data

    def on_set_parameters(self, config, level):
        """ Dynamic reconfigure of the fall checker values."""
        # just pass on to the StandupHandler, as all the variables are located there
        self.blackboard.fall_checker.update_reconfigurable_values(config, level)
        self.blackboard.pickup_accel_threshold = config["pick_up_accel_threshold"]
        return config

    def power_cb(self, msg):
        self.blackboard.is_power_on = msg.data

    def get_state(self):
        return self.blackboard.current_state

    def loop(self):
        """ Keeps updating the DSD and publish its current state.
            All the forwarding of joint goals is directly done in the callbacks to reduce latency. """
        if self.blackboard.shut_down_request and not self.simulation_active:
            self.on_shutdown_hook()
        else:
            loop_start_time = self.node.get_clock().now()
            #can happen in simulation due to bad implementation in rclpy
            if (self.last_loop_start_time != loop_start_time):
                self.last_loop_start_time = loop_start_time
                if self.hcm_deactivated:
                    self.blackboard.current_state = RobotControlState.CONTROLLABLE
                else:
                    self.blackboard.current_time = self.node.get_clock().now()
                    try:
                        self.dsd.update()
                    except IndexError:
                        # this error will happen during shutdown procedure, just ignore it
                        pass

    def on_shutdown_hook(self):
        if not self.blackboard:
            return
        # we got external shutdown, tell it to the DSD, it will handle it
        self.blackboard.shut_down_request = True
        self.node.get_logger().warn("You're stopping the HCM. The robot will sit down and power off its motors.")
        speak("Stopping HCM", self.blackboard.speak_publisher, priority=50)
        # now wait for it finishing the shutdown procedure
        while not self.blackboard.current_state == RobotControlState.HCM_OFF:
            # we still have to update everything
            self.blackboard.current_time = self.node.get_clock().now()
            self.dsd.update()
            self.hcm_state_publisher.publish(self.blackboard.current_state)
            self.node.get_clock().sleep_for(Duration(seconds=0.01))

    def set_last_animation_goal_time(self, time_msg_serialized):
        self.blackboard.last_animation_goal_time = deserialize_message(time_msg_serialized)

    def set_animation_requested(self, animation_requested):
        self.blackboard.animation_requested = animation_requested

    def set_external_animation_running(self, running):
        self.blackboard.external_animation_running = running

    def set_hcm_animation_finished(self, finished):
        self.blackboard.hcm_animation_finished = finished

    def set_record_active(self, active):
        self.blackboard.record_active = active

    def set_last_walking_goal_time(self, time_msg_serialized):
        self.blackboard.last_walking_goal_time = Time.from_msg(deserialize_message(time_msg_serialized))

    def set_last_motor_update_time(self, time_msg_serialized):
        self.blackboard.last_motor_update_time = deserialize_message(time_msg_serialized)

    def set_current_joint_state(self, joint_state_msg_serialized):
        self.blackboard.previous_joint_state = self.blackboard.current_joint_state
        self.blackboard.current_joint_state = deserialize_message(joint_state_msg_serialized)

    def set_cop(self, cop_msg_serialized, left):
        if left:
            self.blackboard.cop_l_msg = deserialize_message(cop_msg_serialized)
        else:
            self.blackboard.cop_r_msg = deserialize_message(cop_msg_serialized)

    def set_pressure_left(self, pressure_msg_serialized):
        msg = deserialize_message(pressure_msg_serialized)
        self.blackboard.last_pressure_update_time = msg.header.stamp
        self.blackboard.pressures[0] = msg.left_front
        self.blackboard.pressures[1] = msg.left_back
        self.blackboard.pressures[2] = msg.right_front
        self.blackboard.pressures[3] = msg.right_back

    def set_pressure_right(self, pressure_msg_serialized):
        msg = deserialize_message(pressure_msg_serialized)
        self.blackboard.last_pressure_update_time = msg.header.stamp
        self.blackboard.pressures[4] = msg.left_front
        self.blackboard.pressures[5] = msg.left_back
        self.blackboard.pressures[6] = msg.right_front
        self.blackboard.pressures[7] = msg.right_back

    def set_imu(self, imu_msg_serialized):
        msg = deserialize_message(imu_msg_serialized)
        self.blackboard.last_imu_update_time = msg.header.stamp

        self.blackboard.accel = numpy.array(
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.blackboard.gyro = numpy.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.blackboard.quaternion = numpy.array(
            ([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))

        self.blackboard.smooth_gyro = numpy.multiply(self.blackboard.smooth_gyro, 0.95) + numpy.multiply(
            self.blackboard.gyro, 0.05)
        self.blackboard.smooth_accel = numpy.multiply(self.blackboard.smooth_accel, 0.99) + numpy.multiply(
            self.blackboard.accel, 0.01)
        self.blackboard.not_much_smoothed_gyro = numpy.multiply(self.blackboard.not_much_smoothed_gyro,
                                                                0.5) + numpy.multiply(self.blackboard.gyro, 0.5)

        self.blackboard.imu_msg = msg
