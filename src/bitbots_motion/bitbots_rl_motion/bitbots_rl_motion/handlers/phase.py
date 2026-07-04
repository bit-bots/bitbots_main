from typing import Optional

import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from bitbots_rl_motion.handlers import Handler


class PhaseHandler(Handler):
    def __init__(self, node: Node):
        self._node = node
        self._use_phase = self._node.get_parameter("phase.use_phase").value
        self._phase = np.array(self._node.get_parameter("phase.initial_phase").value)
        self._control_dt = self._node.get_parameter("phase.control_dt").value
        self._gait_frequency = self._node.get_parameter("phase.gait_frequency").value
        self._phase_dt = 2 * np.pi * self._gait_frequency * self._control_dt

        self._obs_phase = None

        # Optional phase sharing between gait policies (e.g. walk and dribble):
        # the currently active node publishes its phase on the sync topic and a
        # node adopts the last published phase when it takes over, so the gait
        # continues seamlessly across the handover.
        self._sync_topic = str(self._node.get_parameter("phase.sync_topic").value)
        self._external_phase: Optional[np.ndarray] = None
        if self._sync_topic:
            self._sync_pub = self._node.create_publisher(Float64MultiArray, self._sync_topic, 1)
            self._node.create_subscription(Float64MultiArray, self._sync_topic, self._sync_callback, 1)

    def has_data(self):
        return True

    def set_phase(self, new_phase):
        self._phase = new_phase

    def set_obs_phase(self, new_obs_phase):
        self._obs_phase = new_obs_phase

    def get_phase(self):
        return self._phase

    def get_phase_dt(self):
        return self._phase_dt

    def get_obs_phase(self):
        return self._obs_phase

    def check_phase_set(self):
        return self._use_phase

    def _sync_callback(self, msg: Float64MultiArray) -> None:
        self._external_phase = np.array(msg.data)

    def publish_phase(self) -> None:
        """Publish the current phase on the sync topic (no-op when disabled).

        Should only be called by the node while it is actively producing motor
        goals, so the last message on the topic always belongs to the policy
        that ran most recently.
        """
        if not self._sync_topic:
            return
        msg = Float64MultiArray()
        msg.data = [float(value) for value in np.asarray(self._phase).flatten()]
        self._sync_pub.publish(msg)

    def adopt_external_phase(self) -> None:
        """Continue from the last phase published on the sync topic, if any.

        Called on activation so the gait picks up exactly where the previously
        active policy left off instead of restarting from the initial phase.
        """
        if not self._sync_topic or self._external_phase is None:
            return
        if self._external_phase.shape == np.asarray(self._phase).shape:
            self._phase = self._external_phase.copy()
