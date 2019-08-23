#!/usr/bin/env python
import math
import numpy

import rospy
import actionlib

from dynamic_reconfigure.server import Server

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Imu, JointState

from humanoid_league_msgs.msg import Animation as AnimationMsg, PlayAnimationAction, RobotControlState, Speak
from humanoid_league_speaker.speaker import speak
from bitbots_msgs.msg import FootPressure, DynUpAction

from bitbots_msgs.msg import JointCommand
from bitbots_hcm.hcm_dsd.hcm_blackboard import STATE_CONTROLABLE, STATE_WALKING, STATE_ANIMATION_RUNNING, \
    STATE_SHUT_DOWN, STATE_HCM_OFF, STATE_FALLEN, STATE_KICKING
from bitbots_hcm.cfg import hcm_paramsConfig
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard
from dynamic_stack_decider.dsd import DSD
import os


class HardwareControlManager:
    def __init__(self):

        # necessary for on shutdown hook, incase of direct shutdown before finished initilization
        self.blackboard = None

        # --- Initialize Node ---
        log_level = rospy.DEBUG if rospy.get_param("debug_active", False) else rospy.INFO
        rospy.init_node('bitbots_hcm', log_level=log_level, anonymous=False)
        rospy.sleep(0.1)  # Otherwise messages will get lost, bc the init is not finished
        rospy.loginfo("Starting hcm")
        rospy.on_shutdown(self.on_shutdown_hook)

        # stack machine
        self.blackboard = HcmBlackboard()
        self.blackboard.animation_action_client = actionlib.SimpleActionClient('animation', PlayAnimationAction)
        self.blackboard.dynup_action_client = actionlib.SimpleActionClient('dynup', DynUpAction)
        dirname = os.path.dirname(os.path.realpath(__file__)) + "/hcm_dsd"
        self.dsd = DSD(self.blackboard, "/debug/dsd/hcm")
        self.dsd.register_actions(os.path.join(dirname, 'actions'))
        self.dsd.register_decisions(os.path.join(dirname, 'decisions'))
        self.dsd.load_behavior(os.path.join(dirname, 'hcm.dsd'))

        # Publisher / subscriber
        self.joint_goal_publisher = rospy.Publisher('DynamixelController/command', JointCommand, queue_size=1)
        self.hcm_state_publisher = rospy.Publisher('robot_state', RobotControlState, queue_size=1, latch=True)
        self.blackboard.speak_publisher = rospy.Publisher('speak', Speak, queue_size=1)

        rospy.sleep(0.1)  # important to make sure the connection to the speaker is established, for next line
        speak("Starting hcm", self.blackboard.speak_publisher, priority=Speak.HIGH_PRIORITY)

        rospy.Subscriber("imu/data", Imu, self.update_imu, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("foot_pressure", FootPressure, self.update_pressure, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("walking_motor_goals", JointCommand, self.walking_goal_callback, queue_size=1,
                         tcp_nodelay=True)
        rospy.Subscriber("animation", AnimationMsg, self.animation_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("animation_motor_goals", JointCommand, self.dynup_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("head_motor_goals", JointCommand, self.head_goal_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("record_motor_goals", JointCommand, self.record_goal_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("kick_motor_goals", JointCommand, self.kick_goal_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("pause", Bool, self.pause, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("joint_states", JointState, self.joint_state_callback, queue_size=1, tcp_nodelay=True)

        self.dyn_reconf = Server(hcm_paramsConfig, self.reconfigure)

        self.main_loop()

    def pause(self, msg):
        """ Updates the pause/penalty state for the state machine"""
        self.blackboard.penalized = msg.data

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

    def update_pressure(self, msg):
        """Gets new pressure values and writes them to the blackboard"""
        self.blackboard.last_pressure_update_time = msg.header.stamp
        self.blackboard.pressures = [msg.l_l_b, msg.l_l_f, msg.l_r_f, msg.l_r_b, msg.r_l_b, msg.r_l_f, msg.r_r_f,
                                     msg.r_r_b]

    def reconfigure(self, config, level):
        """ Dynamic reconfigure of the fall checker values."""
        # just pass on to the StandupHandler, as all the variables are located there
        self.blackboard.fall_checker.update_reconfigurable_values(config, level)
        return config

    def walking_goal_callback(self, msg):
        self.blackboard.last_walking_goal_time = rospy.Time.now()
        if self.blackboard.current_state == STATE_CONTROLABLE or \
                        self.blackboard.current_state == STATE_WALKING:
            self.joint_goal_publisher.publish(msg)

    def dynup_callback(self, msg):
        # TODO use STATE_GETTING_UP
        if self.blackboard.current_state == STATE_FALLEN:
            self.joint_goal_publisher.publish(msg)

    def head_goal_callback(self, msg):
        if self.blackboard.current_state == STATE_CONTROLABLE or self.blackboard.current_state == STATE_WALKING:
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
        if self.blackboard.current_state == STATE_KICKING:
            # we can perform a kick
            self.joint_goal_publisher.publish(msg)

    def animation_callback(self, msg):
        """ The animation server is sending us goal positions for the next keyframe"""
        self.blackboard.last_animation_goal_time = msg.header.stamp.to_sec()

        if msg.request:
            rospy.loginfo("Got Animation request. HCM will try to get controllable now.")
            # animation has to wait
            # state machine should try to become controllable
            self.blackboard.animation_requested = True
            return

        if msg.first:
            if msg.hcm:
                # comming from ourselves
                # we don't have to do anything, since we must be in the right state
                pass
            else:
                # comming from outside
                # check if we can run an animation now
                if self.blackboard.current_state != STATE_CONTROLABLE:
                    rospy.logwarn("HCM is not controllable, animation refused.")
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
                # this is the last frame, we want to tell the DSD, that we're finished with the animations
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
                # there are sometimes transmittions errors during shutdown due to race conditions
                # there is nothing we can do so just ignore the errors in this case
                try:
                    self.joint_goal_publisher.publish(out_msg)
                except:
                    pass
            else:
                self.joint_goal_publisher.publish(out_msg)

    def joint_state_callback(self, msg):
        self.blackboard.last_motor_update_time = msg.header.stamp
        self.blackboard.current_joint_positions = msg

    def main_loop(self):
        """  """
        rate = rospy.Rate(200)

        while not rospy.is_shutdown() and not self.blackboard.shut_down_request:
            self.blackboard.current_time = rospy.Time.now()
            self.dsd.update()
            self.hcm_state_publisher.publish(self.blackboard.current_state)

            try:
                # catch exeption of moving backwarts in time, when restarting simulator
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn(
                    "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
            except rospy.exceptions.ROSInterruptException:
                exit()

    def on_shutdown_hook(self):
        if not self.blackboard:
            return
        # we got external shutdown, tell it to the DSD, it will handle it
        self.blackboard.shut_down_request = True
        rospy.logwarn("You're stopping the HCM. The robot will sit down and power off its motors.")
        speak("Stopping HCM", self.blackboard.speak_publisher, priority=Speak.HIGH_PRIORITY)
        # now wait for it finishing the shutdown procedure
        while not self.blackboard.current_state == STATE_HCM_OFF:
            # we still have to update everything
            self.blackboard.current_time = rospy.Time.now()
            self.dsd.update()
            self.hcm_state_publisher.publish(self.blackboard.current_state)
            rospy.sleep(0.1)


def main():
    hcm = HardwareControlManager()


if __name__ == "__main__":
    try:
        from bitbots_bringup.nice import Nice

        nice = Nice()
        nice.set_realtime()
    except ImportError:
        rospy.logwarn("Could not import Nice")
    main()
