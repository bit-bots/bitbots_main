import mujoco

from bitbots_mujoco_sim.camera import Camera
from bitbots_mujoco_sim.joint import Joint
from bitbots_mujoco_sim.sensor import FootPressureSensor, Sensor


class Robot:
    """Represents the robot, holding all its components like the camera, joints, and sensors."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, index: int = 0):
        self.index: int = index
        self.camera = Camera(model, data, name=self._get_name("head_camera"))
        self.joints = RobotJoints(
            [
                # --- Right Leg ---
                Joint(model, data, ros_name="RHipYaw", name=self._get_name("RHipYaw")),
                Joint(model, data, ros_name="RHipRoll", name=self._get_name("RHipRoll")),
                Joint(model, data, ros_name="RHipPitch", name=self._get_name("RHipPitch")),
                Joint(model, data, ros_name="RKnee", name=self._get_name("RKnee")),
                Joint(model, data, ros_name="RAnklePitch", name=self._get_name("RAnklePitch")),
                Joint(model, data, ros_name="RAnkleRoll", name=self._get_name("RAnkleRoll")),
                # --- Left Leg ---
                Joint(model, data, ros_name="LHipYaw", name=self._get_name("LHipYaw")),
                Joint(model, data, ros_name="LHipRoll", name=self._get_name("LHipRoll")),
                Joint(model, data, ros_name="LHipPitch", name=self._get_name("LHipPitch")),
                Joint(model, data, ros_name="LKnee", name=self._get_name("LKnee")),
                Joint(model, data, ros_name="LAnklePitch", name=self._get_name("LAnklePitch")),
                Joint(model, data, ros_name="LAnkleRoll", name=self._get_name("LAnkleRoll")),
                # --- Arms ---
                Joint(model, data, ros_name="RShoulderPitch", name=self._get_name("RShoulderPitch")),
                Joint(model, data, ros_name="RShoulderRoll", name=self._get_name("RShoulderRoll")),
                Joint(model, data, ros_name="RElbow", name=self._get_name("RElbow")),
                Joint(model, data, ros_name="LShoulderPitch", name=self._get_name("LShoulderPitch")),
                Joint(model, data, ros_name="LShoulderRoll", name=self._get_name("LShoulderRoll")),
                Joint(model, data, ros_name="LElbow", name=self._get_name("LElbow")),
                # --- Head ---
                Joint(model, data, ros_name="HeadPan", name=self._get_name("HeadPan")),
                Joint(model, data, ros_name="HeadTilt", name=self._get_name("HeadTilt")),
            ]
        )
        self.sensors = RobotSensors(
            [
                # IMU Sensors
                Sensor(model, data, name=self._get_name("gyro"), ros_name="IMU_gyro"),
                Sensor(model, data, name=self._get_name("accelerometer"), ros_name="IMU_accelerometer"),
                Sensor(
                    model,
                    data,
                    name=self._get_name("orientation"),
                    ros_name="IMU_orientation",
                ),  # Global orientation quaternion
                Sensor(model, data, name=self._get_name("position"), ros_name="IMU_position"),  # Global position vector
                # Foot Sensors
                Sensor(model, data, name=self._get_name("l_foot_pos"), ros_name="left_foot_position"),
                Sensor(model, data, name=self._get_name("r_foot_pos"), ros_name="right_foot_position"),
                Sensor(model, data, name=self._get_name("l_foot_global_linvel"), ros_name="left_foot_velocity"),
                Sensor(model, data, name=self._get_name("r_foot_global_linvel"), ros_name="right_foot_velocity"),
            ]
        )

        self.feet_sensors = RobotFeetSensors(
            [
                # left foot
                FootPressureSensor(model, data, name=self._get_name("llb_foot_floor_found"), ros_name="llb"),
                FootPressureSensor(model, data, name=self._get_name("llf_foot_floor_found"), ros_name="llf"),
                FootPressureSensor(model, data, name=self._get_name("lrf_foot_floor_found"), ros_name="lrf"),
                FootPressureSensor(model, data, name=self._get_name("lrb_foot_floor_found"), ros_name="lrb"),
                # right foot
                FootPressureSensor(model, data, name=self._get_name("rlb_foot_floor_found"), ros_name="rlb"),
                FootPressureSensor(model, data, name=self._get_name("rlf_foot_floor_found"), ros_name="rlf"),
                FootPressureSensor(model, data, name=self._get_name("rrf_foot_floor_found"), ros_name="rrf"),
                FootPressureSensor(model, data, name=self._get_name("rrb_foot_floor_found"), ros_name="rrb"),
            ]
        )

    def _get_name(self, base_name: str) -> str:
        return f"robot_{base_name}_{self.index}"

    @property
    def domain(self) -> int:
        return self.index

    @property
    def namespace(self) -> str:
        return f"robot{self.domain}"


class RobotFeetSensors(list[FootPressureSensor]):
    """A list of Robot Foot Pressure Sensors with additional helper methods."""

    @property
    def left(self):
        """Returns a list of left foot pressure sensors."""
        return self.__get_side("l")

    @property
    def right(self):
        """Returns a list of right foot pressure sensors."""
        return self.__get_side("r")

    def get(self, name: str) -> FootPressureSensor:
        """Finds and returns a specific foot pressure sensor by its ROS name."""
        return next(filter(lambda sensor: sensor.ros_name == name, self))

    def __get_side(self, side: str):
        class SideList(list[FootPressureSensor]):
            @property
            def left_back(self) -> FootPressureSensor:
                return self[0]

            @property
            def left_front(self) -> FootPressureSensor:
                return self[1]

            @property
            def right_front(self) -> FootPressureSensor:
                return self[2]

            @property
            def right_back(self) -> FootPressureSensor:
                return self[3]

            @property
            def center_of_pressure(self) -> list[float]:
                # we can take a very small threshold, since simulation gives more accurate values than reality
                threshold = 1
                if self.total_force < threshold:
                    return [0.0, 0.0]

                pos_x, pos_y = 0.085, 0.045
                pressure_diff = (self.left_back.force + self.left_front.force) - (
                    self.right_front.force + self.right_back.force
                )

                return [
                    max(min(pressure_diff * pos_x / self.total_force, pos_x), -pos_x),
                    max(min(pressure_diff * pos_y / self.total_force, pos_y), -pos_y),
                ]

            @property
            def total_force(self) -> float:
                return abs(sum(sensor.force for sensor in self))

        return SideList(
            [
                self.get(f"{side}lb"),
                self.get(f"{side}lf"),
                self.get(f"{side}rf"),
                self.get(f"{side}rb"),
            ]
        )


class RobotSensors(list[Sensor]):
    """A list of Robot Sensors with additional helper methods."""

    def get(self, name: str) -> Sensor:
        """Finds and returns a specific sensor by its ROS name."""
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

    def get(self, name: str) -> Joint:
        """Finds and returns a specific joint by its ROS name."""
        return next(filter(lambda joint: joint.ros_name == name, self))
