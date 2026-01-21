import math
import time
from pathlib import Path
from typing import TYPE_CHECKING, Callable

import mujoco
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from mujoco import viewer
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Float32

from bitbots_msgs.msg import FootPressure, JointCommand
from bitbots_mujoco_sim.domain_bridge_generator import DomainBridgeConfigGenerator
from bitbots_mujoco_sim.robot import Robot

if TYPE_CHECKING:
    from bitbots_mujoco_sim.simulation import RobotSimulation


class Simulation(Node):
    """Manages the MuJoCo simulation state and its components."""

    def __init__(self):
        super().__init__("sim_interface")
        self.package_path = get_package_share_directory("bitbots_mujoco_sim")
        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(self.package_path + "/xml/adult_field.xml")
        self.data: mujoco.MjData = mujoco.MjData(self.model)
        self.robots: list[RobotSimulation] = [
            RobotSimulation(
                self, Robot(self.model, self.data, idx), -1 if len(self._find_robot_indices()) <= 1 else idx + 1
            )
            for idx in self._find_robot_indices()
        ]

        if len(self.robots) > 1:
            bridge_gen = DomainBridgeConfigGenerator(self.robots)
            config_dir = Path(self.package_path) / "config" / "domain_bridges"
            config_path = bridge_gen.generate_config_file(config_dir)
            self.get_logger().info(f"Generated domain bridge config at {config_path}")

        self.time = 0.0
        self.time_message = Time(seconds=0, nanoseconds=0).to_msg()
        self.timestep = self.model.opt.timestep
        self.step_number = 0
        self.real_time_factor = 1.0
        self.clock_publisher = self.create_publisher(Clock, "clock", 1)
        self.create_subscription(Float32, "real_time_factor", self.real_time_factor_callback, 1)

        self.imu_frame_id = self.get_parameter_or("imu_frame", "imu_frame")

        self.camera_optical_frame_id = self.get_parameter_or("camera_optical_frame", "camera_optical_frame")
        self.camera_active = True

        self.events = [
            {"frequency": 1, "handler": self.publish_clock_event},
            {"frequency": 4, "handler": lambda: self.publish(lambda robot: robot.publish_ros_joint_states_event())},
            {"frequency": 4, "handler": lambda: self.publish(lambda robot: robot.publish_imu_event())},
            {"frequency": 4, "handler": lambda: self.publish(lambda robot: robot.publish_pressure_events())},
            {"frequency": 4, "handler": lambda: self.publish(lambda robot: robot.publish_center_of_pressure_events())},
            {"frequency": 32, "handler": lambda: self.publish(lambda robot: robot.publish_camera_event())},
        ]

    def _find_robot_indices(self) -> list[int]:
        """Find all robot instances by looking for bodies named 'robot_torso_X'."""
        body_names = [self.model.body(i).name for i in range(self.model.nbody)]
        robot_bodies = filter(lambda name: name.startswith("robot_torso_"), body_names)
        indices = [int(name.split("_")[-1]) for name in robot_bodies if name.split("_")[-1].isdigit()]
        return sorted(indices)

    def run(self) -> None:
        print("Starting simulation viewer...")
        with viewer.launch_passive(self.model, self.data) as view:
            while view.is_running():
                self.step()
                view.sync()

    def step(self) -> None:
        real_start_time = time.time()
        self.step_number += 1
        self.time += self.timestep
        self.time_message = Time(seconds=int(self.time), nanoseconds=int(self.time % 1 * 1e9)).to_msg()

        mujoco.mj_step(self.model, self.data)

        for event_config in self.events:
            if self.step_number % event_config["frequency"] == 0:
                event_config["handler"]()

        real_end_time = time.time()
        expected_step_time = self.timestep / self.real_time_factor
        time.sleep(max(0.0, expected_step_time - (real_end_time - real_start_time)))

    def real_time_factor_callback(self, msg: Float32) -> None:
        self.real_time_factor = msg.data

    def publish_clock_event(self) -> None:
        clock_msg = Clock()
        clock_msg.clock = self.time_message
        self.clock_publisher.publish(clock_msg)

    def publish(self, executor: Callable[["RobotSimulation"], None]) -> None:
        for robot in self.robots:
            executor(robot)


class RobotSimulation:
    """Holds the simulation state for a single robot instance."""

    def __init__(self, simulation: Simulation, robot: Robot, domain: int):
        self.simulation = simulation
        self.robot = robot
        self.model = simulation.model
        self.data = simulation.data
        self.domain = domain
        self.namespace = f"robot{domain}"
        self.namespace = f"robot{domain}" if domain != -1 else ""

        def _topic(name: str) -> str:
            return "/".join(filter(None, [self.namespace, name]))

        self.node_publishers = {
            "joint_states": self.simulation.create_publisher(JointState, _topic("joint_states"), 1),
            "imu": self.simulation.create_publisher(Imu, _topic("imu/data"), 1),
            "camera_proc": self.simulation.create_publisher(Image, _topic("camera/image_proc"), 1),
            "camera_info": self.simulation.create_publisher(CameraInfo, _topic("camera/camera_info"), 1),
            "foot_pressure_left": self.simulation.create_publisher(
                FootPressure, _topic("foot_pressure_left/filtered"), 1
            ),
            "foot_pressure_right": self.simulation.create_publisher(
                FootPressure, _topic("foot_pressure_right/filtered"), 1
            ),
            "foot_center_of_pressure_left": self.simulation.create_publisher(
                PointStamped, _topic("foot_center_of_pressure_left"), 1
            ),
            "foot_center_of_pressure_right": self.simulation.create_publisher(
                PointStamped, _topic("foot_center_of_pressure_right"), 1
            ),
        }

        self.simulation.create_subscription(
            JointCommand, _topic("DynamixelController/command"), self.joint_command_callback, 1
        )

    def joint_command_callback(self, command: JointCommand) -> None:
        if len(command.positions) != 0:
            for i in range(len(command.joint_names)):
                joint = self.robot.joints.get(command.joint_names[i])
                joint.position = command.positions[i]

    def publish_ros_joint_states_event(self) -> None:
        js = JointState()
        js.name = []
        js.header.stamp = self.simulation.time_message
        js.position = []
        js.velocity = []
        js.effort = []
        for joint in self.robot.joints:
            js.name.append(joint.ros_name)
            js.position.append(joint.position)
            js.velocity.append(joint.velocity)
            js.effort.append(joint.effort)
        self.node_publishers["joint_states"].publish(js)

    def publish_imu_event(self) -> None:
        imu = Imu()
        imu.header.stamp = self.simulation.time_message
        imu.header.frame_id = self.simulation.imu_frame_id
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
            self.robot.sensors.accelerometer.data
        )

        # make sure that acceleration is not completely zero or we will get error in filter.
        # Happens if robot is moved manually in the simulation
        if imu.linear_acceleration.x == 0 and imu.linear_acceleration.y == 0 and imu.linear_acceleration.z == 0:
            imu.linear_acceleration.z = 0.001

        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = self.robot.sensors.gyro.data

        # Use ground-truth orientation from MuJoCo (quaternion: w, x, y, z)
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = self.robot.sensors.orientation.data
        self.node_publishers["imu"].publish(imu)

    def publish_camera_event(self) -> None:
        if not self.simulation.camera_active:
            return

        img = Image()
        img.header.stamp = self.simulation.time_message
        img.header.frame_id = self.simulation.camera_optical_frame_id
        img.encoding = "bgra8"
        img.height = self.robot.camera.height
        img.width = self.robot.camera.width
        img.step = 4 * self.robot.camera.width
        img.data = self.robot.camera.render()
        self.node_publishers["camera_proc"].publish(img)

        cam_info = CameraInfo()
        cam_info.header = img.header
        cam_info.height = self.robot.camera.height
        cam_info.width = self.robot.camera.width

        @staticmethod
        def focal_length_from_fov(fov: float, resolution: int) -> float:
            "Calculate focal length from field of view and resolution."
            return 0.5 * resolution * (math.cos(fov / 2) / math.sin(fov / 2))

        @staticmethod
        def h_fov_to_v_fov(h_fov: float, height: int, width: int) -> float:
            "Convert horizontal FOV to vertical FOV based on image aspect ratio."
            return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

        camera = self.robot.camera

        h_fov = camera.fov
        v_fov = h_fov_to_v_fov(h_fov, camera.height, camera.width)

        f_x, f_y = focal_length_from_fov(h_fov, camera.width), focal_length_from_fov(v_fov, camera.height)
        cx, cy = camera.width / 2.0, camera.height / 2.0
        cam_info.k = [f_x, 0.0, cx, 0.0, f_y, cy, 0.0, 0.0, 1.0]
        cam_info.p = [f_x, 0.0, cx, 0.0, 0.0, f_y, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.node_publishers["camera_info"].publish(cam_info)

    def publish_pressure_events(self) -> None:
        left = FootPressure()
        left.header.stamp = self.simulation.time_message
        left.left_back, left.left_front, left.right_front, left.right_back = [
            -sensor.force for sensor in self.robot.feet_sensors.left
        ]
        self.node_publishers["foot_pressure_left"].publish(left)

        right = FootPressure()
        right.header.stamp = self.simulation.time_message
        right.left_back, right.left_front, right.right_front, right.right_back = [
            -sensor.force for sensor in self.robot.feet_sensors.right
        ]
        self.node_publishers["foot_pressure_right"].publish(right)

    def publish_center_of_pressure_events(self) -> None:
        left = PointStamped()
        left.header.frame_id = "l_foot_frame"
        left.header.stamp = self.simulation.time_message
        left.point.x, left.point.y = self.robot.feet_sensors.left.center_of_pressure
        self.node_publishers["foot_center_of_pressure_left"].publish(left)

        right = PointStamped()
        right.header.frame_id = "r_foot_frame"
        right.header.stamp = self.simulation.time_message
        right.point.x, right.point.y = self.robot.feet_sensors.right.center_of_pressure
        self.node_publishers["foot_center_of_pressure_right"].publish(right)
