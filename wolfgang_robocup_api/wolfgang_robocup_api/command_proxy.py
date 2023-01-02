import rclpy
from bitbots_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from rclpy.node import Node

class CommandProxy(Node):
    def __init__(self):
        super().__init__('joint_command_proxy')
        self.publisher = self.create_publisher(JointState, 'output', 10)
        self.create_subscription(JointCommand, 'input', self.listener_callback, 10)

    def listener_callback(self, msg: JointCommand):
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = msg.joint_names
        joint_states.position = msg.positions
        self.publisher.publish(joint_states)

def main(args=None):
    rclpy.init(args=args)
    command_proxy = CommandProxy()
    rclpy.spin(command_proxy)
    command_proxy.destroy_node()
    rclpy.shutdown()