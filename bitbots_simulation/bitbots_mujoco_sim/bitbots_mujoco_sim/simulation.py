import math
import time

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
from bitbots_mujoco_sim.robot import Robot


class Simulation(Node):
    """Manages the MuJoCo simulation state and its components."""

    def __init__(self):
        super().__init__("sim_interface")
        package_path = get_package_share_directory("bitbots_mujoco_sim")
        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(package_path + "/xml/adult_field.xml")
        self.data: mujoco.MjData = mujoco.MjData(self.model)
        self.robots: list[Robot] = [Robot(self.model, self.data, idx) for idx in self._find_robot_indices()]

        self.time = 0.0
        self.time_message = Time(seconds=0, nanoseconds=0).to_msg()
        self.timestep = self.model.opt.timestep
        self.step_number = 0
        self.real_time_factor = 1.0

        self.create_subscription(JointCommand, "DynamixelController/command", self.joint_command_callback, 1)
        self.create_subscription(Float32, "real_time_factor", self.real_time_factor_callback, 1)

        self.imu_frame_id = self.get_parameter_or("imu_frame", "imu_frame")

        self.camera_optical_frame_id = self.get_parameter_or("camera_optical_frame", "camera_optical_frame")
        self.camera_active = True

        self.node_publishers = {
            "clock": self.create_publisher(Clock, "clock", 1),
            "joint_states": self.create_publisher(JointState, "joint_states", 1),
            "imu": self.create_publisher(Imu, "imu/data_raw", 1),
            "camera_proc": self.create_publisher(Image, "camera/image_proc", 1),
            "camera_info": self.create_publisher(CameraInfo, "camera/camera_info", 1),
            "foot_pressure_left": self.create_publisher(FootPressure, "foot_pressure_left/raw", 1),
            "foot_pressure_right": self.create_publisher(FootPressure, "foot_pressure_right/raw", 1),
            "foot_center_of_pressure_left": self.create_publisher(PointStamped, "cop_l", 1),
            "foot_center_of_pressure_right": self.create_publisher(PointStamped, "cop_r", 1),
        }

        self.js_publisher = self.node_publishers["joint_states"]
        self.clock_publisher = self.node_publishers["clock"]
        self.imu_publisher = self.node_publishers["imu"]
        self.camera_publisher = self.node_publishers["camera_proc"]
        self.camera_info_publisher = self.node_publishers["camera_info"]
        self.pressure_left_publisher = self.node_publishers["foot_pressure_left"]
        self.pressure_right_publisher = self.node_publishers["foot_pressure_right"]
        self.center_of_pressure_left_publisher = self.node_publishers["foot_center_of_pressure_left"]
        self.center_of_pressure_right_publisher = self.node_publishers["foot_center_of_pressure_right"]

        self.events = {
            "clock": {"frequency": 1, "handler": self.publish_clock_event},
            "joint_states": {"frequency": 3, "handler": self.publish_ros_joint_states_event},
            "imu": {"frequency": 3, "handler": self.publish_imu_event},
            "camera": {"frequency": 24, "handler": self.publish_camera_event},
            "pressure": {"frequency": 3, "handler": self.publish_pressure_events},
            "center_of_pressure": {"frequency": 3, "handler": self.publish_center_of_pressure_events},
        }

    # Deprecated, Replace all instances with self.robots to support multiple robots
    @property
    def robot(self) -> Robot:
        return self.robots[0]

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

    def joint_command_callback(self, command: JointCommand) -> None:
        if len(command.positions) != 0:
            for i in range(len(command.joint_names)):
                joint = self.robot.joints.get(command.joint_names[i])
                joint.position = command.positions[i]

    def step(self) -> None:
        real_start_time = time.time()
        self.step_number += 1
        self.time += self.timestep
        self.time_message = Time(seconds=int(self.time), nanoseconds=int(self.time % 1 * 1e9)).to_msg()

        mujoco.mj_step(self.model, self.data)

        for _, event_config in self.events.items():
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

    def publish_ros_joint_states_event(self) -> None:
        js = JointState()
        js.name = []
        js.header.stamp = self.time_message
        js.position = []
        js.effort = []
        for joint in self.robot.joints:
            js.name.append(joint.ros_name)
            js.position.append(joint.position)
            js.velocity.append(joint.velocity)
            js.effort.append(joint.effort)
        self.js_publisher.publish(js)

    def publish_imu_event(self) -> None:
        imu = Imu()
        imu.header.stamp = self.time_message
        imu.header.frame_id = self.imu_frame_id
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
            self.robot.sensors.accelerometer.data
        )

        # make sure that acceleration is not completely zero or we will get error in filter.
        # Happens if robot is moved manually in the simulation
        if imu.linear_acceleration.x == 0 and imu.linear_acceleration.y == 0 and imu.linear_acceleration.z == 0:
            imu.linear_acceleration.z = 0.001

        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = self.robot.sensors.gyro.data

        self.imu_publisher.publish(imu)

    def publish_camera_event(self) -> None:
        if not self.camera_active:
            return

        img = Image()
        img.header.stamp = self.time_message
        img.header.frame_id = self.camera_optical_frame_id
        img.encoding = "bgra8"
        img.height = self.robot.camera.height
        img.width = self.robot.camera.width
        img.step = 4 * self.robot.camera.width
        img.data = self.robot.camera.render()
        self.camera_publisher.publish(img)

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

        self.camera_info_publisher.publish(cam_info)

    def publish_pressure_events(self) -> None:
        left = FootPressure()
        left.header.stamp = self.time_message
        left.left_back, left.left_front, left.right_front, left.right_back = [
            sensor.force for sensor in self.robot.feet_sensors.left
        ]
        self.pressure_left_publisher.publish(left)

        right = FootPressure()
        right.header.stamp = self.time_message
        right.left_back, right.left_front, right.right_front, right.right_back = [
            sensor.force for sensor in self.robot.feet_sensors.right
        ]
        self.pressure_right_publisher.publish(right)

    def publish_center_of_pressure_events(self) -> None:
        left = PointStamped()
        left.header.frame_id = "l_foot_frame"
        left.header.stamp = self.time_message
        left.point.x, left.point.y = self.robot.feet_sensors.left.center_of_pressure
        self.center_of_pressure_left_publisher.publish(left)

        right = PointStamped()
        right.header.frame_id = "r_foot_frame"
        right.header.stamp = self.time_message
        right.point.x, right.point.y = self.robot.feet_sensors.right.center_of_pressure
        print("cop" + str(self.robot.feet_sensors.right.center_of_pressure))
        print("right " + str(right.point.x), right.point.y)
        self.center_of_pressure_right_publisher.publish(right)

    def publish(self, domain_id: int, executor: callable) -> None:
        for robot in self.robots:
            # Set the environment variable here maybe if we don't use the domain bridge
            executor(robot, domain_id)
            # Unset the environment variable here maybe if we don't use the domain bridge
