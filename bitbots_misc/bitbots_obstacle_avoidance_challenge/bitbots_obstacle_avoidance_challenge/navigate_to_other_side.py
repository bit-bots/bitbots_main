import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Duration


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")

        self._package_path = get_package_share_directory("bitbots_obstacle_avoidance_challenge")
        self.pathfinding_pub = self.create_publisher(PoseStamped, "goal_pose", 1)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 1)
        self.get_clock().sleep_for(Duration(seconds=5))
        self.set_initial_pose(-0.75, 3.0, -0.7, 0.7)
        self.get_clock().sleep_for(Duration(seconds=5))
        self.set_goal(-0.75, -3.7, -0.7, 0.7)

    def set_initial_pose(self, x: float, y: float, z, w):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.z = z
        pose_msg.pose.pose.orientation.w = w

        self.initialpose_pub.publish(pose_msg)

    def set_goal(self, x: float, y: float, z, w):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation.z = z
        pose_msg.pose.orientation.w = w

        self.pathfinding_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
