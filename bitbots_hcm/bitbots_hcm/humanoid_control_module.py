#!/usr/bin/env python3
import numpy
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Imu, JointState

from humanoid_league_msgs.msg import Animation as AnimationMsg, PlayAnimationAction, RobotControlState, Audio
from humanoid_league_speaker.speaker import speak
from bitbots_msgs.msg import FootPressure, DynUpAction, KickAction

from bitbots_msgs.msg import JointCommand
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard
from dynamic_stack_decider.dsd import DSD
import os


class HardwareControlManager:
    def __init__(self):
        self.node = Node("HCM")
        # necessary for on shutdown hook, in case of direct shutdown before finished initialization
        self.blackboard = None

        # --- Initialize Node ---
        rclpy.init(args=None)
        self.node.get_clock().sleep_for(
            Duration(seconds=0.1))  # Otherwise messages will get lost, bc the init is not finished
        self.node.get_logger().info("Starting hcm")

        # stack machine
        self.blackboard = HcmBlackboard()
        self.blackboard.animation_action_client = ActionClient(self, PlayAnimationAction, 'animation')
        self.blackboard.dynup_action_client = ActionClient(self, DynUpAction, 'dynup')
        self.blackboard.dynamic_kick_client = ActionClient(self, KickAction, 'dynamic_kick')
        dirname = os.path.dirname(os.path.realpath(__file__)) + "/hcm_dsd"
        self.dsd = DSD(self.blackboard, "debug/dsd/hcm")
        self.dsd.register_actions(os.path.join(dirname, 'actions'))
        self.dsd.register_decisions(os.path.join(dirname, 'decisions'))
        self.dsd.load_behavior(os.path.join(dirname, 'hcm.dsd'))
        self.hcm_deactivated = False

        # Publisher / subscriber
        self.joint_goal_publisher = self.node.create_publisher(JointCommand, 'DynamixelController/command', 1)
        self.hcm_state_publisher = self.node.create_publisher(RobotControlState, 'robot_state', 1)  # todo latch
        self.blackboard.speak_publisher = self.node.create_publisher(Audio, 'speak', 1)

        self.node.get_clock().sleep_for(
            Duration(seconds=0.1))  # important to make sure the connection to the speaker is established, for next line
        speak("Starting hcm", self.blackboard.speak_publisher, priority=50)

        self.node.create_subscription(Imu, "imu/data", self.update_imu, 1)
        self.node.create_subscription(FootPressure, "foot_pressure_left/filtered", self.update_pressure_left, 1)
        self.node.create_subscription(FootPressure, "foot_pressure_right/filtered", self.update_pressure_right, 1)
        self.node.create_subscription(JointCommand, "walking_motor_goals", self.walking_goal_callback, 1)
        self.node.create_subscription(AnimationMsg, "animation", self.animation_callback, 1)
        self.node.create_subscription(JointCommand, "dynup_motor_goals", self.dynup_callback, 1)
        self.node.create_subscription(JointCommand, "head_motor_goals", self.head_goal_callback, 1)
        self.node.create_subscription(JointCommand, "record_motor_goals", self.record_goal_callback, 1)
        self.node.create_subscription(JointCommand, "kick_motor_goals", self.kick_goal_callback, 1)
        self.node.create_subscription(Bool, "pause", self.pause, 1)
        self.node.create_subscription(JointState, "joint_states", self.joint_state_callback, 1)
        self.node.create_subscription(PointStamped, "cop_l", self.cop_l_cb, 1)
        self.node.create_subscription(PointStamped, "cop_r", self.cop_r_cb, 1)
        self.node.create_subscription(Bool, "core/power_switch_status", self.power_cb, 1)
        self.node.create_subscription(Bool, "hcm_deactivate", self.deactivate_cb, 1)

        self.node.add_on_set_parameters_callback(self.on_set_parameters)

        self.main_loop()

    def deactivate_cb(self, msg):
        # we need to make sure the hcm got new sensor messages in between
        self.hcm_deactivated = msg.data

    def pause(self, msg):
        """ Updates the stop state for the state machine"""
        self.blackboard.stopped = msg.data

    def update_imu(self, msg):
        """Gets new IMU values and computes the smoothed values of these"""
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

    def update_pressure_left(self, msg):
        """Gets new pressure values and writes them to the blackboard"""
        self.blackboard.last_pressure_update_time = msg.header.stamp
        self.blackboard.pressures[0] = msg.left_front
        self.blackboard.pressures[1] = msg.left_back
        self.blackboard.pressures[2] = msg.right_front
        self.blackboard.pressures[3] = msg.right_back

    def update_pressure_right(self, msg):
        """Gets new pressure values and writes them to the blackboard"""
        self.blackboard.last_pressure_update_time = msg.header.stamp
        self.blackboard.pressures[4] = msg.left_front
        self.blackboard.pressures[5] = msg.left_back
        self.blackboard.pressures[6] = msg.right_front
        self.blackboard.pressures[7] = msg.right_back

    def on_set_parameters(self, config, level):
        """ Dynamic reconfigure of the fall checker values."""
        # just pass on to the StandupHandler, as all the variables are located there
        self.blackboard.fall_checker.update_reconfigurable_values(config, level)
        self.blackboard.pickup_accel_threshold = config["pick_up_accel_threshold"]
        return config

    def walking_goal_callback(self, msg):
        self.blackboard.last_walking_goal_time = self.node.get_clock().now()
        if self.blackboard.current_state in [RobotControlState.CONTROLLABLE, RobotControlState.WALKING]:
            self.joint_goal_publisher.publish(msg)

    def dynup_callback(self, msg):
        if self.blackboard.current_state in [RobotControlState.STARTUP,
                                             RobotControlState.FALLEN,
                                             RobotControlState.GETTING_UP,
                                             RobotControlState.CONTROLLABLE]:
            self.joint_goal_publisher.publish(msg)

    def head_goal_callback(self, msg):
        if self.blackboard.current_state in [RobotControlState.CONTROLLABLE, RobotControlState.WALKING]:
            # we can move our head
            self.joint_goal_publisher.publish(msg)

    def record_goal_callback(self, msg):
        if msg.joint_names == []:
            # record tells us that its finished
            self.blackboard.record_active = False
        else:
            self.blackboard.record_active = True
            self.joint_goal_publisher.publish(msg)

    def kick_goal_callback(self, msg):
        if self.blackboard.current_state in [RobotControlState.KICKING, RobotControlState.CONTROLLABLE]:
            # we can perform a kick
            self.joint_goal_publisher.publish(msg)

    def animation_callback(self, msg):
        """ The animation server is sending us goal positions for the next keyframe"""
        self.blackboard.last_animation_goal_time = msg.header.stamp.to_sec()

        if msg.request:
            self.node.get_logger().info("Got Animation request. HCM will try to get controllable now.")
            # animation has to wait
            # state machine should try to become controllable
            self.blackboard.animation_requested = True
            return

        if msg.first:
            if msg.hcm:
                # coming from ourselves
                # we don't have to do anything, since we must be in the right state
                pass
            else:
                # coming from outside
                # check if we can run an animation now
                if self.blackboard.current_state != RobotControlState.CONTROLLABLE:
                    self.node.get_logger().warn("HCM is not controllable, animation refused.")
                    return
                else:
                    # we're already controllable, go to animation running
                    self.blackboard.external_animation_running = True

        if msg.last:
            if msg.hcm:
                # This was an animation from the DSD
                self.blackboard.hcm_animation_finished = True
                pass
            else:
                # this is the last frame, we want to tell the DSD that we're finished with the animations
                self.blackboard.external_animation_running = False
                if msg.position is None:
                    # probably this was just to tell us we're finished
                    # we don't need to set another position to the motors
                    return

        # forward positions to motors, if some where transmitted
        if len(msg.position.points) > 0:
            out_msg = JointCommand()
            out_msg.positions = msg.position.points[0].positions
            out_msg.joint_names = msg.position.joint_names
            out_msg.accelerations = [-1] * len(out_msg.joint_names)
            out_msg.velocities = [-1] * len(out_msg.joint_names)
            out_msg.max_currents = [-1] * len(out_msg.joint_names)
            send_torque = False
            if msg.position.points[0].effort:
                out_msg.max_currents = [-x for x in msg.position.points[0].effort]
            if self.blackboard.shut_down_request:
                # there are sometimes transmission errors during shutdown due to race conditions
                # there is nothing we can do so just ignore the errors in this case
                try:
                    self.joint_goal_publisher.publish(out_msg)
                except:
                    pass
            else:
                self.joint_goal_publisher.publish(out_msg)

    def joint_state_callback(self, msg):
        self.blackboard.last_motor_update_time = msg.header.stamp
        self.blackboard.previous_joint_state = self.blackboard.current_joint_state
        self.blackboard.current_joint_state = msg

    def cop_l_cb(self, msg):
        self.blackboard.cop_l_msg = msg

    def cop_r_cb(self, msg):
        self.blackboard.cop_r_msg = msg

    def power_cb(self, msg):
        self.blackboard.is_power_on = msg.data

    def main_loop(self):
        """ Keeps updating the DSD and publish its current state.
            All the forwarding of joint goals is directly done in the callbacks to reduce latency. """
        rate = self.node.create_rate(500, clock=self.node.get_clock())

        while rclpy.ok() and not self.blackboard.shut_down_request:
            if self.hcm_deactivated:
                self.blackboard.current_state = RobotControlState.CONTROLLABLE
                self.hcm_state_publisher.publish(self.blackboard.current_state)
            else:
                self.blackboard.current_time = self.node.get_clock().now()
                try:
                    self.dsd.update()
                    self.hcm_state_publisher.publish(self.blackboard.current_state)
                except IndexError:
                    # this error will happen during shutdown procedure, just ignore it
                    pass

            try:
                # catch exception of moving backwards in time, when restarting simulator
                rate.sleep()
            except:
                self.node.get_logger().warn(
                    "We moved backwards in time. I hope you just reset the simulation. If not there is something "
                    "wrong")
        if not self.node.get_parameter("simulation_active"):
            self.on_shutdown_hook()

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


if __name__ == "__main__":
    hcm = HardwareControlManager()
