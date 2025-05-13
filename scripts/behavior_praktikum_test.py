import enum

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from transitions import Machine


class States(enum.Enum):
    INITIAL = 0
    READY = 1
    SET = 2
    GO_TO_BALL = 3


class StateMachine(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Activate sim time")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)],
        )

        self.state: States
        self.machine = Machine(model=self, states=States, initial=States.INITIAL)
        self.machine.add_transition(trigger="initial", source="*", dest=States.INITIAL)
        self.machine.add_transition(trigger="begin_game", source="*", dest=States.READY)
        self.machine.add_transition("set", "*", States.SET)
        self.machine.add_transition("go_to_ball", States.SET, States.GO_TO_BALL)

        # Create publisher
        self.velocity_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # Create subscriber
        # self.subscription = self.create_subscription(String, "topic", self.listener_callback, 10)
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # match the states

        velocity = Twist()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        match self.state:
            case States.INITIAL | States.SET:
                velocity.linear.x = 0.0
                velocity.linear.y = 0.0
                velocity.angular.z = 0.0
            case States.READY:
                pose.pose.position = -0.25
                pose.pose.position = -1
            case States.GO_TO_BALL:
                self.get_logger().info("State: GO_TO_BALL")
            case _:
                self.get_logger().warn("State: Unknown state")
        self.velocity_publisher_.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
    rclpy.shutdown()
