#!/usr/bin/env python3
"""
Publishes a single JointCommand message for 'head_yaw_joint':
  position  = 0.0
  velocity  = 1.0
  max_torque = 1.0  (effort)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bitbots_msgs.msg import JointCommand


class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        self.publisher_ = self.create_publisher(JointCommand, 'joint_command', 10)

        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Publishing JointCommand for head_yaw_joint at 10 Hz …')

    def timer_callback(self):
        msg = JointCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.from_hcm = False

        msg.joint_names = ['l_ankle_pitch_joint']
        msg.positions = [0.0]
        msg.velocities = [10.0]
        msg.accelerations = [-1.0]   # -1 means "use default" per convention
        msg.max_torques = [0.0]      # effort
        msg.kd = [0.0]              # 0 means "use default" per convention
        msg.kp = [0.0]              # 0 means "use default" per convention

        self.publisher_.publish(msg)
        self.get_logger().debug('Published JointCommand')


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
