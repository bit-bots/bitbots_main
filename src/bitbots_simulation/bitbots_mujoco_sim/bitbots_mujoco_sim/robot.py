import mujoco

from bitbots_mujoco_sim.camera import Camera
from bitbots_mujoco_sim.joint import Joint
from bitbots_mujoco_sim.sensor import Sensor


class Robot:
    """Represents the Pi Plus robot, holding all its components like the camera, joints, and sensors."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, index: int = 0):
        self.index: int = index
        self.camera = Camera(model, data, name=self._get_name("head_camera"))

        def j(ros_name: str) -> Joint:
            return Joint(model, data, ros_name=ros_name, name=self._get_name(ros_name))

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
                Sensor(model, data, name=self._get_name("gyro"), ros_name="IMU_gyro"),
                Sensor(model, data, name=self._get_name("accelerometer"), ros_name="IMU_accelerometer"),
                Sensor(model, data, name=self._get_name("orientation"), ros_name="IMU_orientation"),
                Sensor(model, data, name=self._get_name("position"), ros_name="IMU_position"),
                Sensor(model, data, name=self._get_name("l_foot_pos"), ros_name="left_foot_position"),
                Sensor(model, data, name=self._get_name("r_foot_pos"), ros_name="right_foot_position"),
                Sensor(model, data, name=self._get_name("l_foot_global_linvel"), ros_name="left_foot_velocity"),
                Sensor(model, data, name=self._get_name("r_foot_global_linvel"), ros_name="right_foot_velocity"),
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
