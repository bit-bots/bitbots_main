import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from bitbots_msgs.msg import HeadMode


class CaptureNode(Node):

    def __init__(self):
        super().__init__('capture_node')
        self.publisher_ = self.create_publisher(HeadMode, "head_mode", 10)
        msg = HeadMode()
        msg.head_mode = 1

        self.get_clock().sleep_for(Duration(seconds=5))
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    capture_node = CaptureNode()

    rclpy.spin(capture_node)

    capture_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
