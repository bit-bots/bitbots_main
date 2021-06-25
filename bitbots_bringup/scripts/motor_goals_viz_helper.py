#!/usr/bin/env python3

import argparse

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import JointCommand
from humanoid_league_msgs.msg import Animation
from bitbots_ros_patches.rate import Rate

# List of all joint names. Do not change the order as it is important for Gazebo
JOINT_NAMES = ['HeadPan', 'HeadTilt', 'LShoulderPitch', 'LShoulderRoll', 'LElbow', 'RShoulderPitch',
               'RShoulderRoll', 'RElbow', 'LHipYaw', 'LHipRoll', 'LHipPitch', 'LKnee', 'LAnklePitch',
               'LAnkleRoll', 'RHipYaw', 'RHipRoll', 'RHipPitch', 'RKnee', 'RAnklePitch', 'RAnkleRoll']
JOINT_GOALS = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.7, -1, -0.4, 0, 0, 0, -0.7, 1, 0.4, 0]


class MotorVizHelper:
    def __init__(self):
        # get rid of additional ROS args when used in launch file
        args0 = rospy.myargv()

        parser = argparse.ArgumentParser()
        parser.add_argument("--walking", "-w", help="Directly get walking motor goals", action="store_true")
        parser.add_argument("--animation", "-a", help="Directly get animation motor goals", action="store_true")
        parser.add_argument("--head", help="Directly get head motor goals", action="store_true")
        parser.add_argument("--kick", help="Directly get kick motor goals", action="store_true")
        parser.add_argument("--dynup", help="Directly get Dynup motor goals", action="store_true")
        parser.add_argument("--all", help="Directly get all motor goals", action="store_true")
        parser.add_argument("--gazebo", help="Publish for Gazebo instead of rviz", action="store_true")
        args = parser.parse_args(args0[1:])

        rospy.init_node("motor_viz_helper", anonymous=False)
        if args.gazebo:
            self.joint_publisher = rospy.Publisher('JointGroupController/command', Float64MultiArray, queue_size=10, tcp_nodelay=True)
        else:
            self.joint_publisher = rospy.Publisher('joint_states', JointState, queue_size=10, tcp_nodelay=True)

        if args.walking or args.all:
            rospy.Subscriber("walking_motor_goals", JointCommand, self.joint_command_cb, queue_size=10, tcp_nodelay=True)
        if args.animation or args.all:
            rospy.Subscriber("animation", Animation, self.animation_cb, queue_size=10, tcp_nodelay=True)
        if args.head or args.all:
            rospy.Subscriber("head_motor_goals", JointCommand, self.joint_command_cb, queue_size=10, tcp_nodelay=True)
        if args.kick or args.all:
            rospy.Subscriber("kick_motor_goals", JointCommand, self.joint_command_cb, queue_size=10, tcp_nodelay=True)
        if args.dynup or args.all:
            rospy.Subscriber("dynup_motor_goals", JointCommand, self.joint_command_cb, queue_size=10, tcp_nodelay=True)
        rospy.Subscriber("/DynamixelController/command", JointCommand, self.joint_command_cb, queue_size=10, tcp_nodelay=True)

        self.joint_state_msg = JointState()
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.name = JOINT_NAMES
        self.joint_state_msg.position = JOINT_GOALS
        if args.gazebo:
            self.joint_publisher.publish(self.get_float_array())
        else:
            self.joint_publisher.publish(self.joint_state_msg)

        self.joint_command_msg = JointCommand()
        self.joint_command_msg.joint_names = JOINT_NAMES
        self.joint_command_msg.positions = [0] * 20
        self.joint_command_msg.velocities = [-1] * 20

        rate = Rate(100)
        self.update_time = rospy.Time.now()
        while not rospy.is_shutdown():
            try:
                self.update_joint_states(self.joint_command_msg)
                self.joint_state_msg.header.stamp = rospy.Time.now()
                if args.gazebo:
                    self.joint_publisher.publish(self.get_float_array())
                else:
                    self.joint_publisher.publish(self.joint_state_msg)
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass
            except rospy.exceptions.ROSInterruptException:
                rospy.logwarn('motor_viz_helper: shutting down')
                break

    def joint_command_cb(self, msg: JointCommand):
        self.joint_command_msg.header.stamp = rospy.Time.now()
        for i in range(len(msg.joint_names)):
            if len(msg.positions) != 0:
                # if msg.positions is 0, torque control is probably used.
                # there, the visualization is not implemented yet.
                name = msg.joint_names[i]
                self.joint_command_msg.positions[JOINT_NAMES.index(name)] = msg.positions[i]
                self.joint_command_msg.velocities[JOINT_NAMES.index(name)] = msg.velocities[i]

    def animation_cb(self, msg: Animation):
        self.joint_command_msg.header.stamp = rospy.Time.now()
        for i in range(len(msg.position.joint_names)):
            name = msg.position.joint_names[i]
            self.joint_command_msg.positions[JOINT_NAMES.index(name)] = msg.position.points[0].positions[i]
            self.joint_command_msg.velocities[JOINT_NAMES.index(name)] = -1

    def update_joint_states(self, msg):
        for i in range(len(msg.joint_names)):
            name = msg.joint_names[i]
            if msg.velocities[i] == -1:
                self.joint_state_msg.position[JOINT_NAMES.index(name)] = msg.positions[i]
            else:
                old_pos = self.joint_state_msg.position[JOINT_NAMES.index(name)]
                time_delta = rospy.Time.now() - self.update_time
                time_delta_secs = time_delta.to_sec()
                max_rad = time_delta_secs * msg.velocities[i]
                if msg.positions[i] - old_pos > max_rad:
                    new_pos = old_pos + max_rad
                elif msg.positions[i] - old_pos < -max_rad:
                    new_pos = old_pos - max_rad
                else:
                    new_pos = msg.positions[i]
                self.joint_state_msg.position[JOINT_NAMES.index(name)] = new_pos
        self.update_time = rospy.Time.now()

    def get_float_array(self):
        msg = Float64MultiArray()
        msg.data = self.joint_state_msg.position
        return msg


if __name__ == '__main__':
    helper = MotorVizHelper()

