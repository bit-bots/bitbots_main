import numpy as np
from bitbots_tf_buffer import Buffer
from handlers.gravity_handler import GravityHandler
from handlers.gyro_handler import GyroHandler
from handlers.joint_handler import JointHandler
from handlers.kick_ball_handler import KickBallHandler
from handlers.kick_direction_handler import KickDirectionHandler
from handlers.robot_state_handler import RobotStateHandler
from rclpy.duration import Duration

from bitbots_msgs.msg import JointCommand
from nodes.rl_node import RLNode, create_main


class KickNode(RLNode):
    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="kick_node")

        # Constant kick strength input fed to the policy (see reference implementation).
        self.declare_parameter("kick.strength", 0.5)
        self._kick_strength = float(self.get_parameter("kick.strength").value)

        # publishers
        self._joint_command_pub = self.create_publisher(JointCommand, "kick_motor_goals", 10)

        # shared tf buffer for the ball and kick direction handlers
        self.tf_buffer = Buffer(Duration(seconds=1.0), self)

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._ball_handler = KickBallHandler(self, self.tf_buffer)
        self._kick_direction_handler = KickDirectionHandler(self, self.tf_buffer)
        self._robot_state_handler = RobotStateHandler(self)

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    # observations
    def obs(self):
        return np.hstack(
            [
                self._ball_handler.get_ball_pos(),
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._joint_handler.get_angle_data(),
                self._previous_action.get_previous_action(),
                self._phase.get_obs_phase(),
                self._kick_direction_handler.get_kick_direction(),
                self._kick_strength,
            ]
        ).astype(np.float32)

    # publisher function
    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes (requested/stopped by the behavior via the HCM)
    def allowed_states(self):
        return self._robot_state_handler.is_kickable()
    
    def start(self):
        super().start()
        # Reset previous action to zeros so no leftover state from the last kick influences the first kick step.
        self._previous_action.set_previous_action(np.zeros_like(self._previous_action.get_previous_action()))


main = create_main(KickNode)
