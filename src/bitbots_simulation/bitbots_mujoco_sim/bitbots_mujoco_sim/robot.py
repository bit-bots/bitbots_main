import mujoco
import numpy as np

from bitbots_mujoco_sim.camera import Camera
from bitbots_mujoco_sim.joint import Joint
from bitbots_mujoco_sim.sensor import NoisySensor, QuaternionNoisySensor, Sensor


class Robot:
    """Represents the Pi Plus robot, holding all its components like the camera, joints, and sensors."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        index: int = 0,
        noise_config: dict[str, float] | None = None,
        rng: np.random.Generator | None = None,
    ):
        self.index: int = index
        self._rng = rng or np.random.default_rng()
        self.noise_config = noise_config or {}
        self.camera = Camera(model, data, name=self._get_name("head_camera"))

        def j(ros_name: str) -> Joint:
            return Joint(model, data, ros_name=ros_name, name=self._get_name(ros_name))

        def s(base_name: str, ros_name: str) -> Sensor:
            return Sensor(model, data, name=self._get_name(base_name), ros_name=ros_name)

        def n(base_name: str, ros_name: str, gaussian_key: str, bias_key: str) -> NoisySensor:
            return NoisySensor(
                model,
                data,
                name=self._get_name(base_name),
                ros_name=ros_name,
                gaussian_stddev=float(self.noise_config.get(gaussian_key, 0.0)),
                bias_stddev=float(self.noise_config.get(bias_key, 0.0)),
                rng=self._rng,
            )

        self.joints = RobotJoints(
            [
                # --- Head ---
                j("head_yaw_joint"),
                j("head_pitch_joint"),
                # --- Right Arm ---
                j("r_shoulder_pitch_joint"),
                j("r_shoulder_roll_joint"),
                j("r_upper_arm_joint"),
                j("r_elbow_joint"),
                # --- Left Arm ---
                j("l_shoulder_pitch_joint"),
                j("l_shoulder_roll_joint"),
                j("l_upper_arm_joint"),
                j("l_elbow_joint"),
                # --- Right Leg ---
                j("r_hip_pitch_joint"),
                j("r_hip_roll_joint"),
                j("r_thigh_joint"),
                j("r_calf_joint"),
                j("r_ankle_pitch_joint"),
                j("r_ankle_roll_joint"),
                # --- Left Leg ---
                j("l_hip_pitch_joint"),
                j("l_hip_roll_joint"),
                j("l_thigh_joint"),
                j("l_calf_joint"),
                j("l_ankle_pitch_joint"),
                j("l_ankle_roll_joint"),
            ]
        )
        self.sensors = RobotSensors(
            [
                n("gyro", "IMU_gyro", "gyro_gaussian_stddev", "gyro_bias_stddev"),
                n("accelerometer", "IMU_accelerometer", "accelerometer_gaussian_stddev", "accelerometer_bias_stddev"),
                QuaternionNoisySensor(
                    model,
                    data,
                    name=self._get_name("orientation"),
                    ros_name="IMU_orientation",
                    gaussian_stddev=float(self.noise_config.get("orientation_gaussian_stddev", 0.0)),
                    bias_stddev=float(self.noise_config.get("orientation_bias_stddev", 0.0)),
                    rng=self._rng,
                ),
                s("position", "IMU_position"),
                s("l_foot_pos", "left_foot_position"),
                s("r_foot_pos", "right_foot_position"),
                s("l_foot_global_linvel", "left_foot_velocity"),
                s("r_foot_global_linvel", "right_foot_velocity"),
            ]
        )

    def _get_name(self, base_name: str) -> str:
        return f"robot_{base_name}_{self.index}"

    @property
    def domain(self) -> int:
        return 11 + self.index

    @property
    def namespace(self) -> str:
        return f"robot{self.domain}"


class RobotSensors(list[Sensor]):
    """A list of Robot Sensors with additional helper methods."""

    def get(self, name: str) -> Sensor:
        return next(filter(lambda sensor: sensor.ros_name == name, self))

    @property
    def gyro(self) -> Sensor:
        return self.get("IMU_gyro")

    @property
    def accelerometer(self) -> Sensor:
        return self.get("IMU_accelerometer")

    @property
    def orientation(self) -> Sensor:
        return self.get("IMU_orientation")


class RobotJoints(list[Joint]):
    """A list of Robot Joints with additional helper methods."""

    def get(self, name: str) -> Joint | None:
        return next((j for j in self if j.ros_name == name), None)
