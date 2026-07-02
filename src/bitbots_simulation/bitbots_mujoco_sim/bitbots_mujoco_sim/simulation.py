import time
from collections.abc import Callable
from pathlib import Path

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_directory
from mujoco import viewer
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Float32

from bitbots_msgs.msg import JointCommand
from bitbots_msgs.srv import SimulatorPush
from bitbots_mujoco_sim.robot import Robot


class Simulation(Node):
    """Manages the MuJoCo simulation state and its components."""

    def __init__(self):
        super().__init__("sim_interface")
        self.declare_parameter("world_file", "")
        self.declare_parameter("use_namespace", True)
        self.declare_parameter("web", False)

        # These default value are oriented on some real world data in standstill.
        # They are a bit worse than what was observed there.
        # No systematic bias could be observed, so they are kept at zero as of now.
        self.declare_parameter("imu_accelerometer_gaussian_stddev", 0.05)
        self.declare_parameter("imu_gyro_gaussian_stddev", 0.005)

        self.declare_parameter("imu_accelerometer_bias_stddev", 0.0)
        self.declare_parameter("imu_gyro_bias_stddev", 0.0)

        # The orientation noise IRL seems to be extremely close to 0 in standstill.
        # Further measurements may reveal some larger deviations, but for now we just use 0.
        self.declare_parameter("orientation_gaussian_stddev", 0.0)
        self.declare_parameter("orientation_bias_stddev", 0.0)

        self.declare_parameter("imu_frame", "imu_frame")
        self.declare_parameter("camera_optical_frame", "zed_left_camera_optical_frame")

        self.package_path = get_package_share_directory("bitbots_mujoco_sim")
        world_file = self.get_parameter("world_file").get_parameter_value().string_value
        if not world_file:
            world_file = self._generate_default_world()

        self.use_namespace: bool = self.get_parameter("use_namespace").get_parameter_value().bool_value
        self.imu_accelerometer_gaussian_stddev = (
            self.get_parameter("imu_accelerometer_gaussian_stddev").get_parameter_value().double_value
        )
        self.imu_accelerometer_bias_stddev = (
            self.get_parameter("imu_accelerometer_bias_stddev").get_parameter_value().double_value
        )
        self.imu_gyro_gaussian_stddev = (
            self.get_parameter("imu_gyro_gaussian_stddev").get_parameter_value().double_value
        )
        self.imu_gyro_bias_stddev = self.get_parameter("imu_gyro_bias_stddev").get_parameter_value().double_value
        self.orientation_gaussian_stddev = (
            self.get_parameter("orientation_gaussian_stddev").get_parameter_value().double_value
        )
        self.orientation_bias_stddev = self.get_parameter("orientation_bias_stddev").get_parameter_value().double_value
        self._rng = np.random.default_rng()

        self.model: mujoco.MjModel = mujoco.MjModel.from_xml_path(world_file)
        self.data: mujoco.MjData = mujoco.MjData(self.model)
        self.robots: list[RobotSimulation] = [
            RobotSimulation(
                self,
                Robot(
                    self.model,
                    self.data,
                    idx,
                    noise_config={
                        "gyro_gaussian_stddev": self.imu_gyro_gaussian_stddev,
                        "gyro_bias_stddev": self.imu_gyro_bias_stddev,
                        "accelerometer_gaussian_stddev": self.imu_accelerometer_gaussian_stddev,
                        "accelerometer_bias_stddev": self.imu_accelerometer_bias_stddev,
                        "orientation_gaussian_stddev": self.orientation_gaussian_stddev,
                        "orientation_bias_stddev": self.orientation_bias_stddev,
                    },
                    rng=self._rng,
                ),
            )
            for idx in self._find_robot_indices()
        ]

        self._apply_home_keyframe()

        self.time = 0.0
        self.time_message = Time(seconds=0, nanoseconds=0).to_msg()
        self.timestep = self.model.opt.timestep
        self.step_number = 0
        self.paused = False
        self.real_time_factor = 1.0 / 0.94  # add a small buffer to try to achieve the requested RTF
        self.measured_rtf = 1.0
        self.clock_publisher = self.create_publisher(Clock, "clock", 1)
        self.create_subscription(Float32, "real_time_factor", self.real_time_factor_callback, 1)

        self.imu_frame_id = self.get_parameter("imu_frame").get_parameter_value().string_value
        self.camera_optical_frame_id = self.get_parameter("camera_optical_frame").get_parameter_value().string_value
        self.camera_active = True
        self.early_events = []
        self.events = [
            {"frequency": 1, "handler": self.publish_clock_event},
            {"frequency": 4, "handler": lambda: self.publish(lambda robot: robot.publish_ros_joint_states_event())},
            {"frequency": 4, "handler": lambda: self.publish(lambda robot: robot.publish_imu_event())},
            {"frequency": 32, "handler": lambda: self.publish(lambda robot: robot.publish_camera_event())},
        ]

    def _apply_home_keyframe(self) -> None:
        """Apply the 'home' keyframe joint values to all robots, leaving freejoints untouched.

        The keyframe is defined in pi_plus.xml. Since attached sub-model keyframes are not
        merged into the compiled world model, we load the robot model separately to read them.
        """
        robot_model_path = str(Path(self.package_path) / "xml" / "pi_plus.xml")
        robot_model = mujoco.MjModel.from_xml_path(robot_model_path)
        key_id = mujoco.mj_name2id(robot_model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id < 0:
            return

        # Build a map from bare joint name → keyframe qpos value using the standalone robot model
        home_qpos: dict[str, float] = {}
        home_z: float | None = None
        for i in range(robot_model.njnt):
            jnt = robot_model.joint(i)
            if jnt.type == mujoco.mjtJoint.mjJNT_FREE:
                # qpos layout for freejoint: [x, y, z, qw, qx, qy, qz] — grab z (offset 2)
                home_z = float(robot_model.key_qpos[key_id, jnt.qposadr[0] + 2])
                continue
            home_qpos[jnt.name] = robot_model.key_qpos[key_id, jnt.qposadr[0]]

        for robot_sim in self.robots:
            # Set z spawn height from keyframe while preserving x/y placement and orientation
            if home_z is not None:
                freejoint_name = f"robot_floating_base_joint_{robot_sim.robot.index}"
                freejoint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, freejoint_name)
                if freejoint_id >= 0:
                    self.data.qpos[self.model.joint(freejoint_id).qposadr[0] + 2] = home_z

            for joint in robot_sim.robot.joints:
                # joint.ros_name is the bare name (e.g. "r_hip_pitch_joint") matching the robot model
                value = home_qpos.get(joint.ros_name)
                if value is None:
                    continue
                self.data.qpos[joint.joint_instance.qposadr[0]] = value
                # For position actuators ctrl is the target position — same as the initial qpos
                self.data.ctrl[joint.actuator_instance.id] = value
        mujoco.mj_forward(self.model, self.data)

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

    def _key_callback(self, key: int) -> None:
        # Exceptions in this callback completly deadlock the process.
        if key == ord(" "):
            self.paused = not self.paused

    def run(self) -> None:
        if self.get_parameter("web").get_parameter_value().bool_value:
            self._run_web_viewer()
        else:
            self._run_native_viewer()

    def _run_native_viewer(self) -> None:
        print("Starting simulation viewer...")
        with viewer.launch_passive(
            self.model, self.data, key_callback=self._key_callback, show_left_ui=False, show_right_ui=False
        ) as view:
            # pos="7.709 -7.854 6.511" xyaxes="0.726 0.687 -0.000 -0.357 0.377 0.855
            view.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            view.cam.lookat[0] = 0.0
            view.cam.lookat[1] = 0.0
            view.cam.lookat[2] = 0.0
            view.cam.distance = 12.0
            view.cam.azimuth = 135.0
            view.cam.elevation = -30.0
            while view.is_running():
                if not self.paused:
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
                else:
                    view.sync()

    def _run_web_viewer(self) -> None:
        from mjviser import Viewer

        print("Starting web simulation viewer (mjviser)...")
        # Holds the textured ball overlays, populated by _add_ball_textures below.
        self._ball_overlays: list[tuple[object, int]] = []

        def render(scene) -> None:
            # Default mjviser rendering: push the latest MuJoCo state to the scene.
            scene.update_from_mjdata(self.data)
            # Then keep our textured ball overlays in sync with the simulated balls.
            self._update_ball_overlays(scene)

        viewer = Viewer(
            self.model,
            self.data,
            step_fn=lambda model, data: self.step(),
            render_fn=render,
        )
        self._add_field_textures(viewer.scene.server)
        self._add_ball_textures(viewer.scene.server)
        viewer.run()

    def _add_field_textures(self, server) -> None:
        """Render textured plane geoms (e.g. the soccer field) as flat images in viser.

        mjviser draws plane geoms as a plain grid, which discards the field texture
        defined in the world XML. We re-add each textured plane as a viser image lying
        flat on the ground so the soccer field markings are visible in the web viewer.
        """
        for geom_id in range(self.model.ngeom):
            if self.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_PLANE:
                continue
            matid = int(self.model.geom_matid[geom_id])
            if matid < 0:
                continue
            texid = int(self.model.mat_texid[matid, int(mujoco.mjtTextureRole.mjTEXROLE_RGB)])
            if texid < 0 or self.model.tex_type[texid] != mujoco.mjtTexture.mjTEXTURE_2D:
                continue

            image = self._extract_texture_rgb(texid)
            if image is None:
                continue

            # Plane geom size is (x half-extent, y half-extent, grid spacing).
            half_x = float(self.model.geom_size[geom_id, 0])
            half_y = float(self.model.geom_size[geom_id, 1])
            pos = self.model.geom_pos[geom_id]
            geom_name = self.model.geom(geom_id).name or f"plane_{geom_id}"
            # Parent under mjviser's "/fixed_bodies" frame so the image inherits the
            # camera-tracking offset that frame carries; otherwise the texture stays at
            # the world origin and appears glued to the tracked robot's base link.
            server.scene.add_image(
                f"/fixed_bodies/field_textures/{geom_name}",
                image,
                2 * half_x,
                2 * half_y,
                # Lift slightly above the plane to avoid z-fighting with the grid.
                position=(float(pos[0]), float(pos[1]), float(pos[2]) + 0.001),
                cast_shadow=False,
            )

    def _extract_texture_rgb(self, texid: int) -> np.ndarray | None:
        """Read a MuJoCo 2D texture out of the model as an RGB(A) image array."""
        width = int(self.model.tex_width[texid])
        height = int(self.model.tex_height[texid])
        channels = int(self.model.tex_nchannel[texid])
        if channels not in (3, 4):
            return None
        adr = int(self.model.tex_adr[texid])
        data = self.model.tex_data[adr : adr + width * height * channels]
        # MuJoCo stores textures with a bottom-left origin; flip to the top-left
        # origin expected by image consumers.
        return np.flipud(data.reshape(height, width, channels)).astype(np.uint8)

    def _add_ball_textures(self, server) -> None:
        """Render cube-textured sphere geoms (e.g. the ball) with their texture in viser.

        mjviser draws sphere primitives with a single flat color, so the ball's cube
        texture is lost and it shows up as a plain sphere. For each such geom we add a
        slightly larger sphere mesh whose vertices are colored by sampling the cube map,
        and track its pose every frame via _update_ball_overlays.
        """
        import trimesh

        for geom_id in range(self.model.ngeom):
            if self.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_SPHERE:
                continue
            matid = int(self.model.geom_matid[geom_id])
            if matid < 0:
                continue
            texid = int(self.model.mat_texid[matid, int(mujoco.mjtTextureRole.mjTEXROLE_RGB)])
            if texid < 0 or self.model.tex_type[texid] != mujoco.mjtTexture.mjTEXTURE_CUBE:
                continue

            faces = self._extract_cube_faces(texid)
            if faces is None:
                continue

            # Sit just outside mjviser's flat-colored sphere so it is fully covered.
            radius = float(self.model.geom_size[geom_id, 0]) * 1.01
            mesh = trimesh.creation.icosphere(subdivisions=4, radius=radius)
            directions = mesh.vertices / np.linalg.norm(mesh.vertices, axis=1, keepdims=True)
            colors = self._sample_cube(faces, directions)
            mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=colors)

            geom_name = self.model.geom(geom_id).name or f"sphere_{geom_id}"
            handle = server.scene.add_mesh_trimesh(f"/ball_textures/{geom_name}", mesh)
            self._ball_overlays.append((handle, geom_id))

    def _update_ball_overlays(self, scene) -> None:
        """Move each textured ball overlay to follow its geom's current pose."""
        for handle, geom_id in self._ball_overlays:
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, self.data.geom_xmat[geom_id])
            # scene._scene_offset carries the camera-tracking shift mjviser applies to
            # dynamic geoms, so adding it keeps the ball aligned when tracking is on.
            handle.position = self.data.geom_xpos[geom_id] + scene._scene_offset
            handle.wxyz = quat

    def _extract_cube_faces(self, texid: int) -> np.ndarray | None:
        """Read a MuJoCo cube texture as a (6, w, w, 3) array of RGB faces.

        Faces follow MuJoCo's storage order (+X, -X, +Y, -Y, +Z, -Z).
        """
        width = int(self.model.tex_width[texid])
        height = int(self.model.tex_height[texid])
        channels = int(self.model.tex_nchannel[texid])
        if channels < 3 or height != 6 * width:
            return None
        adr = int(self.model.tex_adr[texid])
        data = self.model.tex_data[adr : adr + width * height * channels]
        return data.reshape(6, width, width, channels)[..., :3].astype(np.uint8)

    @staticmethod
    def _sample_cube(faces: np.ndarray, directions: np.ndarray) -> np.ndarray:
        """Sample a cube map (6, w, w, 3) at unit direction vectors -> (N, 3) colors."""
        size = faces.shape[1]
        x, y, z = directions[:, 0], directions[:, 1], directions[:, 2]
        abs_x, abs_y, abs_z = np.abs(x), np.abs(y), np.abs(z)
        n = directions.shape[0]
        face = np.zeros(n, dtype=int)
        s_coord = np.zeros(n)
        t_coord = np.zeros(n)
        major = np.ones(n)

        # Standard cube-map projection: the dominant axis selects the face, the other
        # two components give the in-face (s, t) coordinates.
        x_dom = (abs_x >= abs_y) & (abs_x >= abs_z)
        pos, neg = x_dom & (x > 0), x_dom & (x <= 0)
        face[pos], s_coord[pos], t_coord[pos], major[pos] = 0, -z[pos], -y[pos], abs_x[pos]
        face[neg], s_coord[neg], t_coord[neg], major[neg] = 1, z[neg], -y[neg], abs_x[neg]

        y_dom = (~x_dom) & (abs_y >= abs_z)
        pos, neg = y_dom & (y > 0), y_dom & (y <= 0)
        face[pos], s_coord[pos], t_coord[pos], major[pos] = 2, x[pos], z[pos], abs_y[pos]
        face[neg], s_coord[neg], t_coord[neg], major[neg] = 3, x[neg], -z[neg], abs_y[neg]

        z_dom = (~x_dom) & (~y_dom)
        pos, neg = z_dom & (z > 0), z_dom & (z <= 0)
        face[pos], s_coord[pos], t_coord[pos], major[pos] = 4, x[pos], -y[pos], abs_z[pos]
        face[neg], s_coord[neg], t_coord[neg], major[neg] = 5, -x[neg], -y[neg], abs_z[neg]

        s = np.clip((s_coord / major + 1.0) * 0.5, 0.0, 1.0)
        t = np.clip((t_coord / major + 1.0) * 0.5, 0.0, 1.0)
        col = np.clip((s * (size - 1)).astype(int), 0, size - 1)
        row = np.clip(((1.0 - t) * (size - 1)).astype(int), 0, size - 1)
        return faces[face, row, col]

    def step(self) -> None:
        self.step_number += 1
        self.time += self.timestep
        self.time_message = Time(seconds=int(self.time), nanoseconds=int(self.time % 1 * 1e9)).to_msg()
        for event_config in self.early_events:
            if self.step_number % event_config["frequency"] == 0:
                event_config["handler"]()
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
            "camera_proc": self.simulation.create_publisher(Image, _topic("zed/zed_node/rgb/image_rect_color"), 1),
            "camera_info": self.simulation.create_publisher(CameraInfo, _topic("zed/zed_node/rgb/camera_info"), 1),
        }

        self.simulation.create_subscription(JointCommand, _topic("joint_command"), self.joint_command_callback, 1)
        self.simulation.create_service(SimulatorPush, _topic("simulator_push"), self.simulator_push_callback)

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
            self.robot.sensors.accelerometer.noisy_data
        )

        if imu.linear_acceleration.x == 0 and imu.linear_acceleration.y == 0 and imu.linear_acceleration.z == 0:
            imu.linear_acceleration.z = 0.001

        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = self.robot.sensors.gyro.noisy_data
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = (
            self.robot.sensors.orientation.noisy_data
        )
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

        camera = self.robot.camera
        cam_info.distortion_model = "plumb_bob"
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        cam_info.k = [camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0]
        cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        cam_info.p = [camera.fx, 0.0, camera.cx, 0.0, 0.0, camera.fy, camera.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.node_publishers["camera_info"].publish(cam_info)

    def simulator_push_callback(
        self, request: SimulatorPush.Request, response: SimulatorPush.Response
    ) -> SimulatorPush.Response:
        force_vector = np.array([request.force.x, request.force.y, request.force.z], dtype=float)
        if request.relative:
            rotation_matrix = np.zeros((3, 3), dtype=float)
            mujoco.mju_quat2Mat(rotation_matrix.reshape(9), self.data.xquat[self.robot.base_body_id])
            force_vector = rotation_matrix @ force_vector

        force = np.concatenate([force_vector, np.zeros(3, dtype=float)])
        self.data.xfrc_applied[self.robot.base_body_id] = force

        def apply_force() -> None:
            self.data.xfrc_applied[self.robot.base_body_id] = force
            if reset_event["end_time"] > self.simulation.time:
                return
            if np.all(self.data.xfrc_applied[self.robot.base_body_id] == force):
                self.data.xfrc_applied[self.robot.base_body_id] = np.zeros(6)
            self.simulation.early_events.remove(reset_event)

        reset_event = {"frequency": 1, "handler": apply_force, "end_time": self.simulation.time + 0.5}
        self.simulation.early_events.append(reset_event)
        return response or SimulatorPush.Response()
