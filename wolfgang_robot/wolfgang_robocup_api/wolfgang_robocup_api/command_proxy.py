import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

from bitbots_msgs.msg import JointCommand


class CommandProxy(Node):
    def __init__(self):
        super().__init__("joint_command_proxy")

        # Get joints max velocities from URDF
        urdf_path = os.path.join(get_package_share_directory("wolfgang_description"), "urdf", "robot.urdf")
        urdf = URDF.from_xml_file(urdf_path)
        self.joints_max_velocities = {
            joint.name: joint.limit.velocity for joint in urdf.joints if joint.type == "revolute"
        }

        self.publisher = self.create_publisher(JointState, "output", 10)
        self.create_subscription(JointCommand, "input", self.listener_callback, 10)

    def listener_callback(self, msg: JointCommand):
        joint_states = JointState()
        joint_states.header.stamp = msg.header.stamp
        joint_states.name = msg.joint_names
        joint_states.position = msg.positions

        # Add custom velocities if available, otherwise use max velocity from URDF
        for i, name in enumerate(msg.joint_names):
            if len(msg.velocities) == 0 or msg.velocities[i] == -1:
                joint_states.velocity.append(self.joints_max_velocities[name])
            else:
                joint_states.velocity.append(msg.velocities[i])

        self.publisher.publish(joint_states)


def main(args=None):
    rclpy.init(args=args)
    command_proxy = CommandProxy()
    rclpy.spin(command_proxy)
    command_proxy.destroy_node()
    rclpy.shutdown()
