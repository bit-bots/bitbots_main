import numpy as np
from handlers.gravity_handler import GravityHandler
from handlers.gyro_handler import GyroHandler
from handlers.joint_handler import JointHandler
from handlers.robot_state_handler import RobotStateHandler

from bitbots_msgs.msg import JointCommand
from nodes.rl_node import RLNode, create_main


class MjLabGetupNode(RLNode):
    """Runtime for the mjlab Pi Plus getup (standup) policy.

    The policy is driven whenever the HCM reports the GETTING_UP robot state.
    """

    def __init__(self):
        # Configures self._phase, self._previous_action.
        super().__init__(node_name="mjlab_getup_node")

        # Dedicated topic so getup goals never collide with walking_motor_goals.
        self._joint_command_pub = self.create_publisher(JointCommand, "getup_motor_goals", 10)

        # Handlers (no command/ball handler: the getup policy has no command input).
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)

        # Loading model.
        model = self.get_parameter("model").value
        self.load_model(model)

    # observations
    def obs(self):
        observation = np.hstack(
            [
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._joint_handler.get_angle_data(),
                self._joint_handler.get_velocity_data(),
                self._previous_action.get_previous_action(),
            ]
        ).astype(np.float32)

        return observation

    # publisher function
    def publisher(self, onnx_pred):
        # The getup policy uses a relative-to-current action space
        # (mjlab RelativeJointPositionAction): target = current_pos + action * scale.
        onnx_pred = np.clip(
            onnx_pred, -1.5, 1.5
        )  # Ensure ONNX output is in expected range before scaling. Out-of-range values
        joint_command = self._joint_handler.get_joint_commands(onnx_pred, relative_to_current=True)

        # Clip the hip to [-0.5, 0.5] to prevent overextension in the getup pose.
        def clipping(current_pos, name, pos):
            if "hip_pitch" in name:
                return np.clip(pos, -1.5, 1.5)
            else:
                return pos

        joint_command.positions = [
            clipping(i, name, pos)
            for i, name, pos in zip(
                self._joint_handler.get_angle_data(), joint_command.joint_names, joint_command.positions
            )
        ]
        self.get_logger().info(f"Publishing getup joint command: {joint_command}")
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes
    def allowed_states(self):
        return self._robot_state_handler.is_getup()

    # getup uses no gait phase
    def _phase_update_hook(self):
        pass


main = create_main(MjLabGetupNode)
