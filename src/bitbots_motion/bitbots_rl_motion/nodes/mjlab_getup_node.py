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
    Motor goals are published to a dedicated topic ("getup_motor_goals") which the
    HCM C++ forwards to the joint controller only while in GETTING_UP — the HCM's
    state-based joint mutex prevents this from fighting walking/head/animation
    goals (which are only forwarded in their own states).

    Observation layout matches the exported ONNX metadata exactly (no command, no
    phase, unlike the walk policy):
        [base_ang_vel(3), projected_gravity(3), joint_pos(22), joint_vel(22), actions(22)]
    The actor was trained with obs_normalization=False, so observations are fed
    raw — there is no separate normalization step here.
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
        joint_command = self._joint_handler.get_joint_commands(onnx_pred, relative_to_current=True)
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes
    def allowed_states(self):
        return self._robot_state_handler.is_getup()

    # getup uses no gait phase
    def _phase_update_hook(self):
        pass


main = create_main(MjLabGetupNode)
