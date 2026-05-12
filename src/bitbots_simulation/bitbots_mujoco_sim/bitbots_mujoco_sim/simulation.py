import math
import time
from pathlib import Path
from typing import TYPE_CHECKING, Callable

import mujoco
from ament_index_python.packages import get_package_share_directory
from mujoco import viewer
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Float32

from bitbots_msgs.msg import JointCommand
from bitbots_mujoco_sim.robot import Robot

if TYPE_CHECKING:
    from bitbots_mujoco_sim.simulation import RobotSimulation


class Simulation(Node):
    """Manages the MuJoCo simulation state and its components."""

    def __init__(self):
        super().__init__("sim_interface")
        self.declare_parameter("world_file", "")
        self.declare_parameter("use_namespace", True)

        self.package_path = get_package_share_directory("bitbots_mujoco_sim")
        world_file = self.get_parameter("world_file").get_parameter_value().string_value
        if not world_file:
            world_file = self._generate_default_world()

        self.use_namespace: bool = self.get_parameter("use_namespace").get_parameter_value().bool_value
        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(world_file)
        self.data: mujoco.MjData = mujoco.MjData(self.model)
        self.robots: list[RobotSimulation] = [
            RobotSimulation(self, Robot(self.model, self.data, idx)) for idx in self._find_robot_indices()
        ]

        self.time = 0.0
        self.time_message = Time(seconds=0, nanoseconds=0).to_msg()
        self.timestep = self.model.opt.timestep
        self.step_number = 0
        self.real_time_factor = 1.0 / 0.94  # add a small buffer to try to achieve the requested RTF
        self.measured_rtf = 1.0
        self.clock_publisher = self.create_publisher(Clock, "clock", 1)
        self.create_subscription(Float32, "real_time_factor", self.real_time_factor_callback, 1)

        self.imu_frame_id = self.get_parameter_or("imu_frame", "imu_frame")
        self.camera_optical_frame_id = self.get_parameter_or("camera_optical_frame", "zed_left_camera_optical_frame")
        self.camera_active = True

        self.events = [
            {"frequency": 1, "handler": self.publish_clock_event},
            {"frequency": 4, "handler": lambda: self.publish(lambda robot: robot.publish_ros_joint_states_event())},
            {"frequency": 4, "handler": lambda: self.publish(lambda robot: robot.publish_imu_event())},
            {"frequency": 32, "handler": lambda: self.publish(lambda robot: robot.publish_camera_event())},
        ]

    def _generate_default_world(self) -> str:
        template_path = Path(self.package_path) / "xml" / "kid_field.xml"
        output_path = Path(self.package_path) / "xml" / "generated_world.xml"
        with open(template_path) as f:
            template = f.read()
        world_xml = (
            template.replace("{{NUM_ROBOTS}}", "1").replace("{{OFFSET}}", "6.0").replace("{{ROBOT_TYPE}}", "piplus")
        )
        with open(output_path, "w") as f:
            f.write(world_xml)
        return str(output_path)

    def _find_robot_indices(self) -> list[int]:
        """Find all robot instances by looking for bodies named 'robot_base_link_X'."""
        body_names = [self.model.body(i).name for i in range(self.model.nbody)]
        return sorted(
            int(name.split("_")[-1])
            for name in body_names
            if name.startswith("robot_base_link_") and name.split("_")[-1].isdigit()
        )

    def run(self) -> None:
        print("Starting simulation viewer...")
        with viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False) as view:
            # pos="7.709 -7.854 6.511" xyaxes="0.726 0.687 -0.000 -0.357 0.377 0.855
            view.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            view.cam.lookat[0] = 0.0
            view.cam.lookat[1] = 0.0
            view.cam.lookat[2] = 0.0
            view.cam.distance = 12.0
            view.cam.azimuth = 135.0
            view.cam.elevation = -30.0
            while view.is_running():
                start_time = time.perf_counter()
                self.step()
                if self.step_number % 16 == 0:  # approx 30 fps
                    view.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, f"RTF: {self.measured_rtf:.2f}x", ""))
                view.sync(state_only=True)
                stop_time = time.perf_counter()
                elapsed = stop_time - start_time
                expected_step_time = self.timestep / self.real_time_factor
                time.sleep(max(0.0, expected_step_time - elapsed))
                total_wall_time = time.perf_counter() - start_time
                if total_wall_time > 0:
                    # weighted ema so it does not fluctuate too much
                    self.measured_rtf = 0.01 * (self.timestep / total_wall_time) + 0.99 * self.measured_rtf

    def step(self) -> None:
        self.step_number += 1
        self.time += self.timestep
        self.time_message = Time(seconds=int(self.time), nanoseconds=int(self.time % 1 * 1e9)).to_msg()
        mujoco.mj_step(self.model, self.data)
        for event_config in self.events:
            if self.step_number % event_config["frequency"] == 0:
                event_config["handler"]()

    def real_time_factor_callback(self, msg: Float32) -> None:
        self.real_time_factor = msg.data / 0.94  # add a small buffer to try to achieve the requested RTF

    def publish_clock_event(self) -> None:
        clock_msg = Clock()
        clock_msg.clock = self.time_message
        self.clock_publisher.publish(clock_msg)

    def publish(self, executor: Callable[["RobotSimulation"], None]) -> None:
        for robot in self.robots:
            executor(robot)


class RobotSimulation:
    """Holds the simulation state for a single robot instance."""

    def __init__(self, simulation: Simulation, robot: Robot):
        self.simulation = simulation
        self.robot = robot
        self.model = simulation.model
        self.data = simulation.data

        def _topic(name: str) -> str:
            return f"{self.namespace}/{name}" if simulation.use_namespace else name

        self.node_publishers = {
            "joint_states": self.simulation.create_publisher(JointState, _topic("joint_states"), 1),
            "imu": self.simulation.create_publisher(Imu, _topic("imu/data"), 1),
            "camera_proc": self.simulation.create_publisher(Image, _topic("zed/zed_node/left/image_rect_color"), 1),
            "camera_info": self.simulation.create_publisher(CameraInfo, _topic("zed/zed_node/left/camera_info"), 1),
        }

        self.simulation.create_subscription(JointCommand, _topic("joint_command"), self.joint_command_callback, 1)

    @property
    def domain(self) -> int:
        return self.robot.domain

    @property
    def namespace(self) -> str:
        return self.robot.namespace

    def joint_command_callback(self, command: JointCommand) -> None:
        if not command.joint_names:
            return
        for i, name in enumerate(command.joint_names):
            joint = self.robot.joints.get(name)
            if joint is None:
                continue
            kp = command.kp[i] if i < len(command.kp) and command.kp[i] > 0 else None
            kd = command.kd[i] if i < len(command.kd) and command.kd[i] > 0 else None
            joint.set_gains(kp, kd)
            joint.position = command.positions[i]

    def publish_ros_joint_states_event(self) -> None:
        js = JointState()
        js.header.stamp = self.simulation.time_message
        js.name = []
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

        if imu.linear_acceleration.x == 0 and imu.linear_acceleration.y == 0 and imu.linear_acceleration.z == 0:
            imu.linear_acceleration.z = 0.001

        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = self.robot.sensors.gyro.data
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
            return 0.5 * resolution * (math.cos(fov / 2) / math.sin(fov / 2))

        @staticmethod
        def h_fov_to_v_fov(h_fov: float, height: int, width: int) -> float:
            return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

        camera = self.robot.camera
        h_fov = camera.fov
        v_fov = h_fov_to_v_fov(h_fov, camera.height, camera.width)
        f_x = focal_length_from_fov(h_fov, camera.width)
        f_y = focal_length_from_fov(v_fov, camera.height)
        cx, cy = camera.width / 2.0, camera.height / 2.0
        cam_info.k = [f_x, 0.0, cx, 0.0, f_y, cy, 0.0, 0.0, 1.0]
        cam_info.p = [f_x, 0.0, cx, 0.0, 0.0, f_y, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.node_publishers["camera_info"].publish(cam_info)
