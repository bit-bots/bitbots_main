import enum

import rclpy
from game_controller_hl_interfaces.msg import GameState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist  # noqa
from rclpy.node import Node
from std_msgs.msg import Empty  # noqa
from transitions.extensions import GraphMachine

from bitbots_msgs.msg import HeadMode  # noqa


# Hier stehen alle states, die eure StateMachine benutzt
class States(enum.Enum):
    INITIAL = 0
    # TODO add more states here


class StateMachine(Node):
    def __init__(self):
        super().__init__("behavior_praktikum")

        self.state: States

        # Hier erstellt ihr eure StateMachine. Dabei wird in "states" das enum von oben angegeben und in "initial" der anfangs state gesetzt.
        self.machine = GraphMachine(
            model=self, states=States, initial=States.INITIAL, title="Behavior", show_conditions=True
        )

        # Wenn ihr eine Transition erstellt, müsst ihr einen Trigger, einen oder mehrere source states angeben, aus denen dann in den dest state gewechselt werden kann.
        # Hier kann aus jedem anderen state in initial gewechselt werden, wenn die Triggerfunktion initial() aufgerufen wird.
        self.machine.add_transition(trigger="initial", source="*", dest=States.INITIAL)
        # TODO add more transitions here

        # Render the state machine as a graph, png can be found in colcon_ws folder
        self.get_graph().draw("behavior.png", prog="dot")

        # Create publisher
        self.velocity_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        # Create subscriber
        self.gamestate_subscription = self.create_subscription(GameState, "gamestate", self.gamestate_callback, 10)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def gamestate_callback(self, msg):
        match msg.game_state:
            case GameState.GAMESTATE_INITIAL:
                self.initial()
            # TODO add more cases for other gamestates and call trigger functions

    # Hier implementiert ihr, was der Roboter in jedem State tun soll.
    def timer_callback(self):
        match self.state:
            case States.INITIAL:
                velocity = Twist()
                velocity.angular.x = -1.0  # stops the robot
                self.velocity_publisher_.publish(velocity)
            # TODO add more cases for your new states
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
