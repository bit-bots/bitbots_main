import mujoco

from bitbots_mujoco_sim.camera import Camera
from bitbots_mujoco_sim.joint import Joint
from bitbots_mujoco_sim.sensor import Sensor


class Robot:
    """Represents the robot, holding all its components like the camera, joints, and sensors."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.camera = Camera(model, data, name="head_camera")
        self.joints = RobotJoints(
            [
                # --- Right Leg ---
                Joint(model, data, ros_name="RHipYaw"),
                Joint(model, data, ros_name="RHipRoll"),
                Joint(model, data, ros_name="RHipPitch"),
                Joint(model, data, ros_name="RKnee"),
                Joint(model, data, ros_name="RAnklePitch"),
                Joint(model, data, ros_name="RAnkleRoll"),
                # --- Left Leg ---
                Joint(model, data, ros_name="LHipYaw"),
                Joint(model, data, ros_name="LHipRoll"),
                Joint(model, data, ros_name="LHipPitch"),
                Joint(model, data, ros_name="LKnee"),
                Joint(model, data, ros_name="LAnklePitch"),
                Joint(model, data, ros_name="LAnkleRoll"),
                # --- Arms ---
                Joint(model, data, ros_name="RShoulderPitch"),
                Joint(model, data, ros_name="RShoulderRoll"),
                Joint(model, data, ros_name="RElbow"),
                Joint(model, data, ros_name="LShoulderPitch"),
                Joint(model, data, ros_name="LShoulderRoll"),
                Joint(model, data, ros_name="LElbow"),
                # --- Head ---
                Joint(model, data, ros_name="HeadPan"),
                Joint(model, data, ros_name="HeadTilt"),
            ]
        )
        self.sensors = RobotSensors(
            [
                # IMU Sensors
                Sensor(model, data, name="gyro", ros_name="IMU_gyro"),
                Sensor(model, data, name="accelerometer", ros_name="IMU_accelerometer"),
                Sensor(
                    model,
                    data,
                    name="orientation",
                    ros_name="IMU_orientation",
                ),  # Global orientation quaternion
                Sensor(model, data, name="position", ros_name="IMU_position"),  # Global position vector
                # Foot Sensors
                Sensor(model, data, name="l_foot_pos", ros_name="left_foot_position"),
                Sensor(model, data, name="r_foot_pos", ros_name="right_foot_position"),
                Sensor(model, data, name="l_foot_global_linvel", ros_name="left_foot_velocity"),
                Sensor(model, data, name="r_foot_global_linvel", ros_name="right_foot_velocity"),
            ]
        )


class RobotSensors(list[Sensor]):
    """A list of Robot Sensors with additional helper methods."""

    def get(self, name: str) -> Joint:
        """Finds and returns a specific joint by its ROS name."""
        return next(filter(lambda joint: joint.ros_name == name, self))

    @property
    def gyro(self) -> Sensor:
        return self.get("IMU_gyro")

    @property
    def accelerometer(self) -> Sensor:
        return self.get("IMU_accelerometer")


class RobotJoints(list[Joint]):
    """A list of Robot Joints with additional helper methods."""

    def get(self, name: str) -> Joint:
        """Finds and returns a specific joint by its ROS name."""
        return next(filter(lambda joint: joint.ros_name == name, self))
