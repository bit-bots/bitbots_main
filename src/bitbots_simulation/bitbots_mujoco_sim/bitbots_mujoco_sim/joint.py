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
        self.actuator_instance: int = model.actuator(self.name.replace("_joint_", "_"))

        aid = self.actuator_instance.id
        self._default_kp: float = float(model.actuator_gainprm[aid, 0])
        self._default_kd: float = float(-model.actuator_biasprm[aid, 2])

    def set_gains(self, kp: float | None = None, kd: float | None = None) -> None:
        """Dynamically update actuator PD gains. Falls back to model defaults for None or non-positive values."""
        aid = self.actuator_instance.id
        kp = kp if (kp is not None and kp > 0) else self._default_kp
        kd = kd if (kd is not None and kd > 0) else self._default_kd
        self.model.actuator_gainprm[aid, 0] = kp
        self.model.actuator_biasprm[aid, 1] = -kp
        self.model.actuator_biasprm[aid, 2] = -kd

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
