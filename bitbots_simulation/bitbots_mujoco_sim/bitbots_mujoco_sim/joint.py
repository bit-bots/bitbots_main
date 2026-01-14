import mujoco


class Joint:
    """Represents a single controllable joint and associated actuator in the MuJoCo simulation."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        ros_name: str,
        name: str | None = None,
    ):
        self.model: mujoco.MjModel = model
        self.data: mujoco.MjData = data
        self.ros_name: str = ros_name
        self.name: str = name if name is not None else ros_name
        self.joint_instance: int = model.joint(self.name)
        self.actuator_instance: int = model.actuator(self.name)

    @property
    def position(self) -> float:
        """Gets the current joint position (angle) in radians."""
        return self.data.qpos[self.joint_instance.qposadr[0]]

    @position.setter
    def position(self, value: float) -> None:
        """Sets the position target for the joint's position actuator."""
        self.data.ctrl[self.actuator_instance.id] = value

    @property
    def velocity(self) -> float:
        """Gets the current joint velocity in rad/s."""
        return self.data.qvel[self.joint_instance.dofadr[0]]

    @property
    def effort(self) -> float:
        """Gets the current effort (force/torque) applied by the position actuator."""
        return self.data.actuator_force[self.actuator_instance.id]
