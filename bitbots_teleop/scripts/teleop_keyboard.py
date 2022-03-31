#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard
import math
import os
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import JointCommand

import sys, select, termios, tty
from rclpy.action import ActionClient
from bitbots_msgs.action import Kick
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf_transformations import quaternion_from_euler
import argparse

# the following encodes walkready poses for various robots. could be done more nicely via some config

__walkready_joints__ = {
    "wolfgang":
        [("HeadPan", 0.0),
         ("HeadTilt", 0.0),
         ("LShoulderPitch", math.radians(75.27)),
         ("LShoulderRoll", 0.0),
         ("LElbow", math.radians(35.86)),
         ("RShoulderPitch", math.radians(-75.58)),
         ("RShoulderRoll", 0.0),
         ("RElbow", math.radians(-36.10)),
         ("LHipYaw", -0.0112),
         ("LHipRoll", 0.0615),
         ("LHipPitch", 0.4732),
         ("LKnee", 1.0058),
         ("LAnklePitch", -0.4512),
         ("LAnkleRoll", 0.0625),
         ("RHipYaw", 0.0112),
         ("RHipRoll", -0.0615),
         ("RHipPitch", -0.4732),
         ("RKnee", -1.0059),
         ("RAnklePitch", 0.4512),
         ("RAnkleRoll", -0.0625)],
    "robotis_op2":
        [("LElbow", math.radians(-60)),
         ("RElbow", math.radians(60)),
         ("LShoulderPitch", math.radians(120.0)),
         ("RShoulderPitch", math.radians(-120.0))],
    "op3":
        [("l_el", math.radians(-140)),
         ("r_el", math.radians(140)),
         ("l_sho_pitch", math.radians(-135)),
         ("r_sho_pitch", math.radians(135)),
         ("l_sho_roll", math.radians(-90)),
         ("r_sho_roll", math.radians(90))],
    "nao":
        [("LShoulderPitch", 1.57),
         ("RShoulderPitch", 1.57),
         ('LShoulderRoll', 0.3),
         ('RShoulderRoll', -0.3)],
    "rfc":
        [("LeftElbow", math.radians(-90.0)),
         ("RightElbow", math.radians(90.0)),
         ("LeftShoulderPitch [shoulder]", math.radians(45.0),),
         ("RightShoulderPitch [shoulder]", math.radians(-45.0))],
    "chape":
        [("leftElbowYaw", math.radians(-160)),
         ("rightElbowYaw", math.radians(160)),
         ("leftShoulderPitch[shoulder]", math.radians(75.27)),
         ("rightShoulderPitch[shoulder]", math.radians(75.58)),
         ("leftShoulderYaw", math.radians(-75.58)),
         ("rightShoulderYaw", math.radians(75.58))],
    "mrl_hsl":
        [("Shoulder-L [shoulder]", math.radians(60.0)),
         ("Shoulder-R [shoulder]", math.radians(-60.0)),
         ("UpperArm-L", math.radians(10.0)),
         ("UpperArm-R", math.radians(-10.0)),
         ("LowerArm-L", math.radians(-135.0)),
         ("LowerArm-R", math.radians(135.0))],
    "nugus":
        [("left_elbow_pitch", math.radians(-120)),
         ("right_elbow_pitch", math.radians(-120)),
         ("left_shoulder_pitch [shoulder]", math.radians(120)),
         ("right_shoulder_pitch [shoulder]", math.radians(120)),
         ("left_shoulder_roll", math.radians(20)),
         ("right_shoulder_roll", math.radians(-20))],
    "sahrv74":
        [("left_shoulder_pitch [shoulder]", math.radians(60.0)),
         ("right_shoulder_pitch [shoulder]", math.radians(60.0)),
         ("left_shoulder_roll", math.radians(10.0)),
         ("right_shoulder_roll", math.radians(10.0)),
         ("left_elbow", math.radians(-135.0)),
         ("right_elbow", math.radians(-135.0))],
    "bez":
        [("left_arm_motor_0 [shoulder]", math.radians(0)),
         ("right_arm_motor_0 [shoulder]", math.radians(0)),
         ("left_arm_motor_1", math.radians(170)),
         ("right_arm_motor_1", math.radians(170))]
}

__velocity__ = 5.0
__accelerations__ = -1.0
__max_currents__ = -1.0

msg = """
BitBots Teleop
--------------
Walk around:            Move head:
    q    w    e         u    i    o
    a    s    d         j    k    l
                        m    ,    .  

q/e: turn left/right    k: zero head position
a/d: left/rigth         i/,: up/down
w/s: forward/back       j/l: left/right
                        u/o/m/.: combinations     

Controls increase / decrease with multiple presses.
SHIFT increases with factor 10

y: kick left forward   Y: walk kick left forward
c: kick right forward  C: walk kick right forward
<: side kick left 1    >: side kick left 2
v: side kick right 1   V: side kick right 2
x: kick center forward X: kick center backward
b: kick left backward  n: kick right backward
B: kick left outward   N: kick right outward

f: full stop           F: play walkready animation
r: reset robot in simulation
R: reset ball in simulation

CTRL-C to quit




"""

moveBindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
    'W': (10, 0, 0),
    'S': (-10, 0, 0),
    'A': (0, 10, 0),
    'D': (0, -10, 0),
    'Q': (0, 0, 10),
    'E': (0, 0, -10),
}
headBindings = {
    'u': (1, 1),
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'm': (-1, 1),
    ',': (-1, 0),
    '.': (-1, -1),
    'U': (10, 10),
    'I': (10, 0),
    'O': (10, -10),
    'J': (0, 10),
    'L': (0, -10),
    'M': (-10, 10),
    ';': (-10, 0),
    ':': (-10, -10)
}


class TeleopKeyboard(Node):

    def __init__(self):
        # create node
        super().__init__("TeleopKeyboard")

        self.settings = termios.tcgetattr(sys.stdin)

        parser = argparse.ArgumentParser()
        parser.add_argument('--robot-type',
                            help="Which robot type is used {wolfgang, op2, op3, nao, rfc, chape, mrl_hsl}",
                            default="wolfgang")
        args, unknown = parser.parse_known_args()
        robot_type = args.robot_type
        if robot_type not in ["wolfgang", "op2", "op3", "nao", "rfc", "chape", "mrl_hsl"]:
            self.get_logger().fatal(
                f"Robot type {robot_type} not known. Should be one of [wolfgang, op2, op3, nao, rfc, chape, mrl_hsl]")
            exit()

        # generate walkready command
        joint_names = []
        joint_positions = []
        for joint_tuple in __walkready_joints__[robot_type]:
            joint_names.append(joint_tuple[0])
            joint_positions.append(joint_tuple[1])
        self.walkready = JointCommand(
            joint_names=joint_names,
            velocities=[__velocity__] * len(joint_names),
            accelerations=[__accelerations__] * len(joint_names),
            max_currents=[__max_currents__] * len(joint_names),
            positions=joint_positions)

        # Walking Part
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.head_pan_pos = 0
        self.head_tilt_pos = 0

        self.x_speed_step = 0.01
        self.y_speed_step = 0.01
        self.turn_speed_step = 0.01

        self.x = 0
        self.y = 0
        self.a_x = -1
        self.th = 0
        self.status = 0

        # Head Part
        self.create_subscription(JointState, "joint_states", self.joint_state_cb, 1)
        if self.get_parameter("use_sim_time").get_parameter_value().bool_value:
            self.head_pub = self.create_publisher(JointCommand, "head_motor_goals", 1)
        else:
            self.head_pub = self.create_publisher(JointCommand, "DynamixelController/command", 1)
        self.head_msg = JointCommand()
        self.head_msg.max_currents = [float(-1)] * 2
        self.head_msg.velocities = [float(5)] * 2
        self.head_msg.accelerations = [float(40)] * 2
        self.head_msg.joint_names = ['HeadPan', 'HeadTilt']
        self.head_msg.positions = [float(0)] * 2

        self.head_pan_step = 0.05
        self.head_tilt_step = 0.05

        self.walk_kick_pub = self.create_publisher(Bool, "kick", 1)
        self.joint_pub = self.create_publisher(JointCommand, "DynamixelController/command", 1)

        self.reset_robot = self.create_client(Empty, "/reset_pose")
        self.reset_ball = self.create_client(Empty, "/reset_ball")

        print(msg)

        self.frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"
        self.client = ActionClient(self, Kick, 'dynamic_kick')

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def generate_kick_goal(self, x, y, direction):
        kick_goal = Kick.Goal()
        kick_goal.header.stamp = self.get_clock().now().to_msg()
        kick_goal.header.frame_id = self.frame_prefix + "base_footprint"
        kick_goal.ball_position.x = float(x)
        kick_goal.ball_position.y = float(y)
        kick_goal.ball_position.z = float(0)
        x, y, z, w = quaternion_from_euler(0, 0, direction)
        kick_goal.kick_direction.x = float(x)
        kick_goal.kick_direction.y = float(y)
        kick_goal.kick_direction.z = float(z)
        kick_goal.kick_direction.w = float(w)
        kick_goal.kick_speed = float(1)
        return kick_goal

    def joint_state_cb(self, msg):
        if "HeadPan" in msg.name and "HeadTilt" in msg.name:
            self.head_pan_pos = msg.position[msg.name.index("HeadPan")]
            self.head_tilt_pos = msg.position[msg.name.index("HeadTilt")]

    def loop(self):
        try:
            while True:
                key = self.getKey()
                if key in moveBindings.keys():
                    self.x += moveBindings[key][0] * self.x_speed_step

                    self.x = round(self.x, 2)
                    self.y += moveBindings[key][1] * self.y_speed_step
                    self.y = round(self.y, 2)
                    self.th += moveBindings[key][2] * self.turn_speed_step
                    self.th = round(self.th, 2)
                    self.a_x = 0
                elif key in headBindings.keys():
                    self.head_msg.positions[0] = self.head_pan_pos + headBindings[key][1] * self.head_pan_step
                    self.head_msg.positions[1] = self.head_tilt_pos + headBindings[key][0] * self.head_tilt_step
                    self.head_pub.publish(self.head_msg)
                elif key == 'k' or key == 'K':
                    # put head back in init
                    self.head_msg.positions[0] = 0
                    self.head_msg.positions[1] = 0
                    self.head_pub.publish(self.head_msg)
                elif key == 'y':
                    # kick left forward
                    self.client.send_goal_async(self.generate_kick_goal(0.2, 0.1, 0))
                elif key == '<':
                    # kick left side ball left
                    self.client.send_goal_async(self.generate_kick_goal(0.2, 0.1, -1.57))
                elif key == '>':
                    # kick left side ball center
                    self.client.send_goal_async(self.generate_kick_goal(0.2, 0, -1.57))
                elif key == 'c':
                    # kick right forward
                    self.client.send_goal_async(self.generate_kick_goal(0.2, -0.1, 0))
                elif key == 'v':
                    # kick right side ball right
                    self.client.send_goal_async(self.generate_kick_goal(0.2, -0.1, 1.57))
                elif key == 'V':
                    # kick right side ball center
                    self.client.send_goal_async(self.generate_kick_goal(0.2, 0, 1.57))
                elif key == "x":
                    # kick center forward
                    self.client.send_goal_async(self.generate_kick_goal(0.2, 0, 0))
                elif key == "X":
                    # kick center backwards
                    self.client.send_goal_async(self.generate_kick_goal(-0.2, 0, 0))
                elif key == "b":
                    # kick left backwards
                    self.client.send_goal_async(self.generate_kick_goal(-0.2, 0.1, 0))
                elif key == "n":
                    # kick right backwards
                    self.client.send_goal_async(self.generate_kick_goal(-0.2, -0.1, 0))
                elif key == "B":
                    # kick left backwards
                    self.client.send_goal_async(self.generate_kick_goal(0, 0.14, -1.57))
                elif key == "N":
                    # kick right backwards
                    self.client.send_goal_async(self.generate_kick_goal(0, -0.14, 1.57))
                elif key == 'Y':
                    # kick left walk
                    self.walk_kick_pub.publish(False)
                elif key == 'C':
                    # kick right walk
                    self.walk_kick_pub.publish(True)
                elif key == 'F':
                    # play walkready animation
                    self.walkready.header.stamp = self.get_clock().now().to_msg()
                    self.joint_pub.publish(self.walkready)
                elif key == 'r':
                    # reset robot in sim
                    try:
                        self.reset_robot.call_async(Empty.Request())
                    except:
                        pass
                elif key == 'R':
                    # reset ball in sim
                    try:
                        self.reset_ball.call_async(Empty.Request())
                    except:
                        pass
                elif key == 'f':
                    # complete walk stop
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.a_x = -1
                    self.th = 0
                else:
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.a_x = 0
                    self.th = 0
                    if (key == '\x03'):
                        self.a_x = -1
                        break

                twist = Twist()
                twist.linear.x = float(self.x)
                twist.linear.y = float(self.y)
                twist.linear.z = float(0)
                twist.angular.x = float(self.a_x)
                twist.angular.y = float(0)
                twist.angular.z = float(self.th)
                self.pub.publish(twist)
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                print(
                    f"x:    {self.x}          \ny:    {self.y}          \nturn: {self.th}          \n\n")

        except Exception as e:
            print(e)

        finally:
            print("\n")
            twist = Twist()
            twist.linear.x = float(0)
            twist.linear.y = float(0)
            twist.linear.z = float(0)
            twist.angular.x = float(0)
            twist.angular.y = float(0)
            twist.angular.z = float(0)
            self.pub.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    rclpy.init(args=None)
    node = TeleopKeyboard()
    # necessary so that sleep in loop() is not blocking
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.loop()

    node.destroy_node()
    rclpy.shutdown()
