import enum

import rclpy
from game_controller_hl_interfaces.msg import GameState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Empty
from transitions.extensions import GraphMachine


class States(enum.Enum):
    INITIAL = 0
    READY = 1
    SET = 2
    GO_TO_BALL = 3


class StateMachine(Node):
    def __init__(self):
        super().__init__("behavior_praktikum")
        self.get_logger().info("Activate sim time")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)],
        )

        # Create state machine
        self.state: States
        self.machine = GraphMachine(
            model=self, states=States, initial=States.INITIAL, title="Behavior", show_conditions=True
        )
        self.machine.add_transition(trigger="initial", source="*", dest=States.INITIAL)
        self.machine.add_transition(trigger="begin_game", source="*", dest=States.READY)
        self.machine.add_transition("set", "*", States.SET)
        self.machine.add_transition("go_to_ball", States.SET, States.GO_TO_BALL)

        # Render the state machine as a graph
        self.get_graph().draw("behavior.png", prog="dot")

        # Create publisher
        self.velocity_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cancel_path_planning_publisher_ = self.create_publisher(Empty, "/pathfinding/cancel", 10)

        # Create subscriber
        self.subscription = self.create_subscription(GameState, "gamestate", self.listener_callback, 10)
        self.vision_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/ball_position_relative_filtered", self.vision_callback, 10
        )
        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.has_kickoff = True

    def listener_callback(self, msg):
        self.get_logger().info(f"Received game state: {msg.game_state}")
        match msg.game_state:
            case GameState.GAMESTATE_INITIAL:
                self.initial()
            case GameState.GAMESTATE_READY:
                self.begin_game()
            case GameState.GAMESTATE_SET:
                self.set()
            case GameState.GAMESTATE_PLAYING:
                self.go_to_ball()
        self.has_kickoff = msg.has_kick_off

    # def vision_callback(self, msg):

    def timer_callback(self):
        # match the states

        match self.state:
            case States.INITIAL | States.SET:
                self.cancel_path_planning_publisher_.publish(Empty())
                velocity = Twist()
                velocity.angular.x = -1.0
                self.velocity_publisher_.publish(velocity)
            case States.READY:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                if self.has_kickoff:
                    pose.pose.position.x = -0.25
                else:
                    pose.pose.position.x = -1.0
                pose.pose.orientation.w = 1.0
                self.goal_pose_publisher_.publish(pose)
            case States.GO_TO_BALL:
                self.get_logger().info("State: GO_TO_BALL")
            case _:
                self.get_logger().warn("State: Unknown state")


if __name__ == "__main__":
    rclpy.init()
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
    rclpy.shutdown()
