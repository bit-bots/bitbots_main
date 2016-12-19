# -*- coding: utf8 -*-
import json
import time

import rospy
from bitbots_animation.srv import AnimationFrame
from bitbots_common.pose.pypose import PyPose as Pose
from dynamic_reconfigure.server import Server
from humanoid_league_msgs.msg import MotionState
from humanoid_league_msgs.msg import Speak
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from bitbots_animation.src.bitbots_animation.animation import parse
from bitbots_motion.src.abstract_state_machine import VALUES, switch_motor_power
from bitbots_motion.src.motion_state_machine import MotionStateMachine, STATE_RECORD, STATE_GETTING_UP, STATE_SOFT_OFF, \
    STATE_CONTROLABLE, STATE_ANIMATION_RUNNING, STATE_WALKING, STATE_PENALTY_ANIMATION, AnimationRunning
from .cfg import bitbots_motion_params


class Motion(object):
    def __init__(self, dieflag, standupflag, softoff_flag, softstart, start_test):
        rospy.init_node('bitbots_motion', anonymous=False)
        rospy.Subscriber("/IMU", Imu, self.update_imu)
        rospy.Subscriber("/MotorCurrentPosition", JointState, self.update_current_pose)
        rospy.Subscriber("/WalkingMotorGoals", JointTrajectory, self.walking_goal_callback)
        rospy.Subscriber("/HeadMotorGoals", JointTrajectory, self.head_goal_callback)
        rospy.Subscriber("/RecordMotorGoals", JointTrajectory, self.record_goal_callback)
        rospy.Subscriber("/pause", bool, self.pause)
        self.animation_keyframe_service = rospy.Service("animation_key_frame", AnimationFrame, self.keyframe_callback)
        self.joint_goal_publisher = rospy.Publisher('/MotionMotorGoals', JointState, queue_size=10)
        self.state_publisher = rospy.Publisher('/MotionState', JointState, queue_size=10)
        self.speak_publisher = rospy.Publisher('/Speak', Speak, queue_size=10)
        self.dyn_reconf = Server(bitbots_motion_params, self.reconfigure)

        self.accel = (0, 0, 0)
        self.gyro = (0, 0, 0)
        self.smooth_accel = (0, 0, 0)
        self.smooth_gyro = (0, 0, 0)
        self.not_much_smoothed_gyro = (0, 0, 0)

        self.robo_pose = Pose()
        self.goal_pose = Pose()
        self.startup_time = time.time()
        self.first_run = True
        self.walking_goal = None
        self.head_goal = None

        self.animation_running = False  # animation request from animation server
        self.animation_request_time = 0  # time we got the animation request

        self.state_machine = MotionStateMachine(dieflag, standupflag, softoff_flag, softstart, start_test,
                                                self.state_publisher)

        self.update_forever()

    def pause(self, msg):
        VALUES.penalized = msg

    def update_imu(self, msg):
        # todo check if this is not switched
        self.gyro = msg.linear_velocity
        self.accel = msg.angular_velocity

        # todo check if this is needed by something else in the software
        # todo make smoothing factors reconfigurable
        # Remember smoothed gyro values
        # Used for falling detectiont, smooth_gyro is to late but peaks have to be smoothed anyway
        # increasing smoothing -->  later detection
        # decreasing smoothing --> more false positives
        self.smooth_gyro = self.smooth_gyro * 0.9 + self.gyro * 0.1  ###gyro
        self.smooth_accel = self.smooth_accel * 0.9 + self.accel * 0.1  ###accel
        self.not_much_smoothed_gyro = self.not_much_smoothed_gyro * 0.5 + self.gyro * 0.5

    def update_current_pose(self, msg):
        VALUES.last_client_update = msg.header.stamp
        self.robo_pose.setPositions(msg.positions)
        self.robo_pose.setSpeed(msg.velocities)

    def publish_motion_state(self):
        msg = MotionState()
        msg.state = self.state_machine.get_current_state()
        self.state_publisher.publish(msg)

    def reconfigure(self, config, level):
        # just pass on to the StandupHandler, as the variables are located there
        VALUES.stand_up_handler.update_reconfigurable_values(config, level)

    def publish_motor_goals(self):
        # we can only handle one point and not a full trajectory
        msg = JointTrajectoryPoint()
        msg.positions = self.goal_pose.get_positions()
        msg.velocities = self.goal_pose.get_speeds()
        traj_msg = JointTrajectory()
        traj_msg.points = []
        traj_msg.points.append(msg)
        traj_msg.header.stamp = rospy.Time.now()
        self.joint_goal_publisher.publish(traj_msg)

    def walking_goal_callback(self, msg):
        self.walking_goal = msg
        VALUES.walking_active = True

    def head_goal_callback(self, msg):
        self.head_goal = msg

    def record_goal_callback(self, msg):
        if msg is None:
            # record tells us that its finished
            VALUES.record = False
        else:
            VALUES.record = True
            self.joint_goal_publisher.publish(msg)

    def keyframe_callback(self, req: AnimationFrame):
        # the animation server is sending us goal positions
        VALUES.last_request = req.header.stamp
        self.animation_request_time = rospy.Time.now()
        if req.first_frame:
            self.animation_running = True
            VALUES.external_animation_finished = False
            if req.force:
                # force to run directly the animation
                # this is a bit hacky, but I don't know a better solution
                self.state_machine.set_state(AnimationRunning())
                # todo walking stop signal
                # todo check if motor power active
            else:
                if self.state_machine.get_current_state() != STATE_CONTROLABLE:
                    # animation has to wait
                    # state machine should try to become controllable
                    VALUES.animation_requested = True
                    return False
                else:
                    # we're already controllable, go to animation running
                    VALUES.external_animation_play = True

        if req.last_frame:
            # this is the last frame, we want to tell the state machine, that we're finished with the animations
            self.animation_running = False
            VALUES.external_animation_finished = True

        # sending keyframe positions to hardware
        self.joint_goal_publisher.publish(req.positions)
        return True

    def update_forever(self):
        """ Ruft :func:`update_once` in einer Endlosschleife auf """
        iteration = 0
        duration_avg = 0
        start = rospy.Time.now()

        while True:
            self.update_once()

            # Count to get the update frequency
            iteration += 1
            if iteration < 100:
                continue

            if duration_avg > 0:
                duration_avg = 0.5 * duration_avg + 0.5 * (rospy.Time.now() - start)
            else:
                duration_avg = (rospy.Time.now() - start)

            rospy.loginfo("Updates/Sec %f", iteration / duration_avg)
            iteration = 0
            start = time.time()

    def update_once(self):
        # check if we're still walking
        if rospy.Time.now() - self.walking_goal.header.stamp > 0.5:
            VALUES.walking_active = False

        # let statemachine run
        self.state_machine.evaluate()

        # now do corresponding actions depending on state of state machine

        if self.state_machine.is_record():
            # we are currently in record mode
            # the motor goals are set directly in the callback method, so we don't have to do anything
            return

        if self.animation_running and rospy.Time.now() - self.animation_request_time < 1:
            # we are currently running an animation
            # the motor goals are set directly in the callback method, so we don't have to do anything
            return

        if self.state_machine.is_walking():
            # we're currently walking
            # set positions from first point of trajectory
            point = self.walking_goal.points[0]
            self.goal_pose.set_positions(point.positions)
            self.goal_pose.set_speed(point.velocities)

        if not self.state_machine.is_penalized():
            # we can move our head
            point = self.head_goal.points[0]
            self.goal_pose.set_positions(point.positions)
            self.goal_pose.set_speed(point.velocities)

        if self.state_machine.is_shutdown():
            # the motion has to shutdown, we close the node
            exit("Motion shutdown")


def switch_motor_power(state):
    """ Calling service from CM730 to turn motor power on or off"""
    # todo set motor ram here if turned on, bc it lost it
    rospy.wait_for_service("switch_motor_power")
    power_switch = rospy.ServiceProxy("switch_motor_power", SwitchMotorPower)
    try:
        response = power_switch(state)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    # wait for motors
    rospy.sleep(1)
