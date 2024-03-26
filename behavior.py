#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, Twist
from ros2_numpy import numpify
from tf_transformations import euler_from_quaternion


class Behavior(Node):
    def __init__(self):
        super().__init__("schnupperstudium_behavior")
        self.get_logger().info("Initializing...")

        # The robot's position in the world
        # Zero is in the middle of the field
        # x is positive towards the opponent's goal
        # y is positive to the left
        # z is positive upwards (not used for this example)
        # The unit is meters
        self.position = Vector3(x=0, y=0, z=0)

        # The robot's orientation in the world
        # Zero is facing the opponent's goal
        # Positive is counter-clockwise if you look from above
        # The unit is degrees
        self.orientation = 0

        # Ball position in the world
        # Same coordinate system as the robot (see above)
        self.ball_position = Vector3(x=0, y=0, z=0)

        # Ball confidence
        # A value between 0 and 1 describing how confident the robot is about the ball's position
        # 0 means the robot is not confident at all
        # 1 means the robot is very confident
        self.ball_confidence = 0

        # You can ignore all code below this line for now

        # A callback function that updates the robot's pose
        def pose_callback(msg: PoseWithCovarianceStamped):
            self.position = msg.pose.pose.position
            self.orientation = math.degrees(euler_from_quaternion(numpify(msg.pose.pose.orientation))[2] % math.tau)

        # Create a subscriber the robot's pose
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            "/pose_with_covariance",
            pose_callback,
            10
        )

        # A callback function that updates the ball's position
        def ball_callback(msg: PoseWithCovarianceStamped):
            self.ball_position = msg.pose.pose.position
            self.ball_confidence = 1 - 1/(1 + (msg.pose.covariance[0] + msg.pose.covariance[7]) / 2)

        # Create a subscriber for the ball's position
        self.ball_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            "/ball_position_relative_filtered",
            ball_callback,
            10
        )

        # Create a cmd_vel publisher
        # This publisher is used to send commands to the robots walk engine
        self.walk_command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Create a timer that calls the run method every 100ms
        self.create_timer(0.1, self.run)

    def walk(self, x: float, y: float, theta: float):
        """
        Send a walk command to the robot's walk engine.

        :param x: The forward speed in m/s
        :param y: The sideways speed in m/s
        :param theta: The angular speed in degrees/s
        """
        self.walk_command_publisher.publish(Twist(
            linear=Vector3(x=float(x), y=float(y)),
            angular=Vector3(z=math.radians(float(theta)))
        ))

    def stop(self):
        """
        Send a stop command to the robot's walk engine.
        """
        self.walk_command_publisher.publish(Twist(angular=Vector3(x=-1.0)))

    def run(self):
        """
        This method is called every 100ms. You can put your code here, so it gets executed periodically.
        """
        self.get_logger().info("Executing...")
        self.get_logger().info(f"Robot position: {self.position.x:.2f}m, {self.position.y:.2f}m | Orientation: {self.orientation:.2f}° | Ball position: {self.ball_position.x:.2f}m, {self.ball_position.y:.2f}m | Ball confidence: {self.ball_confidence:.2f}")

        # --------------------->    Your code goes here    <-------------------

        # Example: Stand still
        self.stop()

        # Example: Walk forward
        # self.walk(0.1, 0, 0.5)


# This is the main entry point of the program, you can ignore it for now
if __name__ == "__main__":
    rclpy.init()
    behavior = Behavior()
    try:
        rclpy.spin(behavior)
    except KeyboardInterrupt:
        pass
    behavior.destroy_node()
