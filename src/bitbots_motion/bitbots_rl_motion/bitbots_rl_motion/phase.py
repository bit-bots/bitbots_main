import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32


class Phase:
    def __init__(self, node: Node):
        self._node = node
        self._use_phase = self._node.get_parameter("phase.use_phase").value
        self._phase = np.array(self._node.get_parameter("phase.initial_phase").value)
        self._control_dt = self._node.get_parameter("phase.control_dt").value
        self._gait_frequency = self._node.get_parameter("phase.gait_frequency").value
        self._phase_dt = 2 * np.pi * self._gait_frequency * self._control_dt

        self._obs_phase = None
        # Latest phase published by the other (currently active) policy. Used to keep the
        # phase of the inactive policy aligned so that a hot swap between walk and kick is smooth.
        self._external_phase = None

        if self._use_phase:
            # Shared phase topic between the walk and kick policy.
            self._phase_pub = self._node.create_publisher(Float32, "rl_phase", 1)
            self._node.create_subscription(Float32, "rl_phase", self._external_phase_callback, 1)

    def _external_phase_callback(self, msg: Float32):
        self._external_phase = msg.data

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

    def advance(self):
        """Advance the phase by one control step and wrap it to [-pi, pi)."""
        phase_tp1 = self._phase + self._phase_dt
        self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi
        return self._phase

    def publish_phase(self):
        """Publish the leading phase so the inactive policy can stay in sync (hot swap)."""
        self._phase_pub.publish(Float32(data=float(self._phase[0])))

    def sync_from_external(self):
        """Initialize our phase from the active policy's published phase (hot swap)."""
        if self._external_phase is None:
            return
        p = self._external_phase
        self._phase = np.array([p, (p + 2 * np.pi) % (2 * np.pi) - np.pi], dtype=np.float32)
