#!/usr/bin/env python3

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.duration import Duration

from biped_interfaces.msg import Phase


class PhaseFromTransform(Node):
    def __init__(self):
        super().__init__("phase_from_transform")

        # Parameters
        self.declare_parameter("l_sole_frame", "l_sole")
        self.declare_parameter("r_sole_frame", "r_sole")
        self.declare_parameter("base_frame", "base_link")

        self.l_sole_frame = self.get_parameter("l_sole_frame").value
        self.r_sole_frame = self.get_parameter("r_sole_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # TF2
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.phase_pub = self.create_publisher(Phase, "walk_support_state", 1)

        # Timer
        self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        try:
            # Get transforms for both feet relative to base
            l_transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.l_sole_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
            r_transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.r_sole_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )

            # Compare z-heights to determine phase
            l_height = l_transform.transform.translation.z
            r_height = r_transform.transform.translation.z

            msg = Phase()
            msg.header.stamp = self.get_clock().now().to_msg()

            # Determine phase based on relative height
            height_diff = abs(l_height - r_height)
            if height_diff < 0.005:  # Both feet on ground
                msg.phase = Phase.DOUBLE_STANCE
            elif l_height < r_height:  # Left foot lower (on ground)
                msg.phase = Phase.LEFT_STANCE  # RIGHT_SWING
            else:  # Right foot lower (on ground)
                msg.phase = Phase.RIGHT_STANCE  # LEFT_SWING

            self.phase_pub.publish(msg)

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PhaseFromTransform()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
