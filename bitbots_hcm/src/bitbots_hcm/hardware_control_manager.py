#!/usr/bin/env python
import math
import numpy

import rospy
import actionlib

from dynamic_reconfigure.server import Server

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Imu

from humanoid_league_msgs.msg import Animation as AnimationMsg, PlayAnimationAction, RobotControlState, Speak
from humanoid_league_speaker.speaker import speak

from bitbots_ros_control.msg import JointCommand, JointTorque
from bitbots_hcm.hcm_stack_machine.hmc_connector import HcmStateMachine, STATE_CONTROLABLE, AnimationRunning, STATE_WALKING, STATE_ANIMATION_RUNNING, STATE_SHUT_DOWN
from bitbots_hcm.cfg import hcm_paramsConfig
from bitbots_connector.connector import AbstractConnector
from bitbots_hcm.hcm_stack_machine import HcmConnector

class HardwareControlManager:
    def __init__(self):

        # stack machine
        self.connector = HcmConnector()        
        self.connector.animation_action_client = actionlib.SimpleActionClient('animation', PlayAnimationAction)
        self.stack_machine = StackMachine(self.connector, "debug_hcm_stack_machine")

        # --- Initialize Node ---
        log_level = rospy.DEBUG if rospy.get_param("debug_active", False) else rospy.INFO
        rospy.init_node('bitbots_hcm', log_level=log_level, anonymous=False)
        rospy.sleep(0.1)  # Otherwise messages will get lost, bc the init is not finished
        rospy.loginfo("Starting hcm")

        self.joint_goal_publisher = rospy.Publisher('DynamixelController/command', JointCommand, queue_size=1)
        self.hcm_state_publisher = rospy.Publisher('robot_state', RobotControlState, queue_size=1, latch=True)
        self.connector.speak_publisher = rospy.Publisher('speak', Speak, queue_size=1)

        rospy.sleep(0.1)  # important to make sure the connection to the speaker is established, for next line
        speak("Starting hcm", self.connector.speak_publisher, priority=Speak.HIGH_PRIORITY)
        
        rospy.Subscriber("imu/data", Imu, self.update_imu, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("walking_motor_goals", JointCommand, self.walking_goal_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("animation", AnimationMsg, self.animation_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("head_motor_goals", JointCommand, self.head_goal_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("record_motor_goals", JointCommand, self.record_goal_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("pause", Bool, self.pause, queue_size=1, tcp_nodelay=True)    

        self.dyn_reconf = Server(hcm_paramsConfig, self.reconfigure)

        self.main_loop()

    def pause(self, msg):
        """ Updates the pause/penalty state for the state machine"""
        self.connector.penalized = msg.data

    def update_imu(self, msg):
        """Gets new IMU values and computes the smoothed values of these"""
        self.connector.last_imu_update = msg.header.stamp

        self.connector.accel = numpy.array([msg.linear_acceleration.x , msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.connector.gyro = numpy.array([msg.angular_velocity.x, msg.angular_velocity.y , msg.angular_velocity.z])
        self.connector.quaternion = numpy.array(([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))

        self.connector.smooth_gyro = numpy.multiply(self.connector.smooth_gyro, 0.95) + numpy.multiply(self.connector.gyro, 0.05)
        self.connector.smooth_accel = numpy.multiply(self.connector.smooth_accel, 0.99) + numpy.multiply(self.connector.accel, 0.01)
        self.connector.not_much_smoothed_gyro = numpy.multiply(self.connector.not_much_smoothed_gyro, 0.5) + numpy.multiply(self.connector.gyro, 0.5)    

    def reconfigure(self, config, level):
        """ Dynamic reconfigure of the fall checker values."""
        # just pass on to the StandupHandler, as all the variables are located there
        self.connector.fall_checker.update_reconfigurable_values(config, level)
        return config

    def walking_goal_callback(self, msg):
        self.connector.last_walking_goal_time = rospy.Time.now()
        if self.connector.current_state == STATE_CONTROLABLE or \
                        self.connector.current_state == STATE_WALKING:
            self.joint_goal_publisher.publish(msg)

    def head_goal_callback(self, msg):
        if self.connector.current_state == STATE_CONTROLABLE or self.connector.current_state == STATE_WALKING:
            # we can move our head
            self.joint_goal_publisher.publish(msg)

    def record_goal_callback(self, msg):
        if msg is None:
            # record tells us that its finished
            self.connector.record_active = False
        else:
            self.connector.record_active = True
            self.joint_goal_publisher.publish(msg)

    def animation_callback(self, msg):
        """ The animation server is sending us goal positions for the next keyframe"""        
        self.connector.last_animation_goal_time = msg.header.stamp.to_sec() #TODO nano sec?

        if msg.first:
            self.animation_running = True
            if msg.hcm:
                # comming from ourselves
                pass
            else:
                # comming from outside
                if self.connector.current_state != STATE_CONTROLABLE:
                    rospy.logwarn("Motion is not controllable, animation refused.")
                    # animation has to wait
                    # state machine should try to become controllable
                    self.connector.animation_requested = True
                    return
                else:
                    # we're already controllable, go to animation running
                    self.connector.external_animation_running = True

        if msg.last:
            if msg.hcm:
                # This was an animation from the state machine
                pass
            else:
                # this is the last frame, we want to tell the state machine, that we're finished with the animations
                self.animation_running = False
                self.connector.external_animation_running= False
                if msg.position is None:
                    # probably this was just to tell us we're finished
                    # we don't need to set another position to the motors
                    return

        # forward positions to motors, if some where transmitted
        if len(msg.position.points) > 0:
            self.joint_goal_publisher.publish(msg.position)

    def main_loop(self):
        """  """
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.stack_machine.evaluate()            

            try:
                # catch exeption of moving backwarts in time, when restarting simulator
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
            except rospy.exceptions.ROSInterruptException:
                exit()

        # we got external shutdown, tell it to the state machine, it will handle it
        self.connector.shut_down = True
        rospy.logwarn("You're stopping the Hcm. The robot will sit down and power off its motors.")
        # now wait for it finishing the shutdown procedure
        while not self.connector.current_state == STATE_SHUT_DOWN:
            # we still have to update everything
            self.update_once()
            rospy.sleep(0.01)        

def main():
    hcm = HardwareControlManager()

if __name__ == "__main__":
    try:
        from bitbots_common.nice import Nice
        nice = Nice()
        nice.set_realtime()
    except ImportError:
        rospy.logwarn("Could not import Nice")
    main()
