import asyncio
import json
import threading
import time

import rclpy
import websockets
from biped_interfaces.msg import Phase
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import SetBool
from transforms3d.euler import euler2quat, quat2euler


class WebSocketClient:
    """Run a WebSocket client in its own asyncio loop (background thread) with auto-reconnect."""

    def __init__(self, uri, reconnect_delay=5.0):
        self.uri = uri
        self.reconnect_delay = reconnect_delay
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        self.connection = None
        self._ensure_connection()

    async def _connect(self):
        ws = await websockets.connect(
            self.uri,
            ping_interval=20,
            ping_timeout=10,
        )
        await ws.send("droidpad")
        resp = await ws.recv()
        if resp != "droidup":
            raise RuntimeError("Handshake failed")
        return ws

    def _run(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result()

    def _ensure_connection(self):
        """Try to establish connection, retrying until success."""
        while self.connection is None:
            try:
                self.connection = self._run(self._connect())
                print(f"[WebSocketClient] Connected to {self.uri}")
            except Exception as e:
                print(f"[WebSocketClient] Connection failed: {e}, retrying in {self.reconnect_delay}s")
                time.sleep(self.reconnect_delay)

    def send(self, msg: str):
        try:
            return self._run(self.connection.send(msg))
        except Exception:
            self.connection = None
            self._ensure_connection()
            return self._run(self.connection.send(msg))

    def recv(self):
        try:
            return self._run(self.connection.recv())
        except Exception:
            self.connection = None
            self._ensure_connection()
            return self._run(self.connection.recv())

    def close(self):
        if self.connection:
            return self._run(self.connection.close())


class X02Control(Node):
    def __init__(self, server_host="192.168.254.100", server_port=8765):
        super().__init__("x02_control")

        # Joint mappings
        self.joint_name_map = {
            "muT": "LHipYaw",
            "msL": "LHipRoll",
            "mhL": "LHipPitch",
            "mkL": "LKnee",
            "maL": "LAnklePitch",
            "mbT": "RHipYaw",
            "msR": "RHipRoll",
            "mhR": "RHipPitch",
            "mkR": "RKnee",
            "maR": "RAnklePitch",
        }
        self.inv_joint_name_map = {v: k for k, v in self.joint_name_map.items()}
        fake_joints = [
            "LAnkleRoll",
            "RAnkleRoll",
            "LShoulderPitch",
            "RShoulderPitch",
            "LShoulderRoll",
            "RShoulderRoll",
            "LElbow",
            "RElbow",
            "HeadTilt",
            "HeadPan",
        ]

        # WebSocket client with reconnect
        self.server_uri = f"ws://{server_host}:{server_port}"
        self.ws = WebSocketClient(self.server_uri)

        # State
        self.robot_cmd = {
            "vel_x": 0.0,
            "vel_y": 0.0,
            "vel_yaw": 0.0,
            "base_roll": 0.0,
            "base_pitch": 0.0,
            "base_yaw": 0.0,
            "base_z": 0.0,
            "loco_mode": "loco_walk_and_stand",  # loco_stop, loco_walk
            "arm_mode": "init",  # init, hello, shake
        }
        self.robot_status = {
            "joint_q": [0.0] * 10,
            "joint_dq": [0.0] * 10,
            "joint_tau": [0.0] * 10,
            "zero_command": 0,  # int
            "walk_phase": 0.0,
            "imu_euler_roll": 0.0,
            "imu_euler_pitch": 0.0,
            "imu_euler_yaw": 0.0,
            "imu_gyro_x": 0.0,
            "imu_gyro_y": 0.0,
            "imu_gyro_z": 0.0,
            "imu_acc_x": 0.0,
            "imu_acc_y": 0.0,
            "imu_acc_z": 0.0,
            "vel_x": 0.0,  # velocity estimator estimates
            "vel_y": 0.0,
            "vel_z": 0.0,
        }
        # Joints
        self.n_fake_joints = len(fake_joints)
        self.joint_names = list(self.joint_name_map.values()) + fake_joints

        self.walk_odom = Odometry()
        self.walk_odom.header.frame_id = "odom"
        self.walk_odom.child_frame_id = "base_link"
        self.walk_odom.pose.pose.position.z = 0.8

        # ROS interfaces
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.phase_pub = self.create_publisher(Phase, "/walk_support_state", 10)
        self.vel_estimator_pub = self.create_publisher(Twist, "/velocity_estimator", 10)
        self.walk_odom_pub = self.create_publisher(Odometry, "/walk_engine_odometry", 10)

        self.set_torque_service = self.create_service(SetBool, "set_torque", self.set_torque_cb)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)

        # Periodic update
        self.update_timer = self.create_timer(0.01, self.update)

    def update(self):
        """ROS timer callback → send command + receive status."""
        try:
            self.ws.send(json.dumps(self.robot_cmd))
            response = self.ws.recv()
            self.robot_status.update(json.loads(response))
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")
            return

        # Publish JointState
        time = self.get_clock().now()
        stamp = time.to_msg()
        js = JointState()
        js.header.stamp = (time - rclpy.time.Duration(seconds=0.01)).to_msg()
        js.name = self.joint_names
        js.position = self.robot_status.get("joint_q", [0.0] * 10) + [0.0] * self.n_fake_joints
        js.velocity = self.robot_status.get("joint_dq", [0.0] * 10) + [0.0] * self.n_fake_joints
        js.effort = self.robot_status.get("joint_tau", [0.0] * 10) + [0.0] * self.n_fake_joints
        self.joint_state_pub.publish(js)

        # Publish Imu
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_frame"
        r, p, y = (
            self.robot_status.get("imu_euler_roll", 0.0),
            self.robot_status.get("imu_euler_pitch", 0.0),
            self.robot_status.get("imu_euler_yaw", 0.0),
        )
        q = euler2quat(r, p, y)
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = q
        imu.angular_velocity.x = self.robot_status.get("imu_gyro_x", 0.0)
        imu.angular_velocity.y = self.robot_status.get("imu_gyro_y", 0.0)
        imu.angular_velocity.z = self.robot_status.get("imu_gyro_z", 0.0)
        imu.linear_acceleration.x = self.robot_status.get("imu_acc_x", 0.0)
        imu.linear_acceleration.y = self.robot_status.get("imu_acc_y", 0.0)
        imu.linear_acceleration.z = self.robot_status.get("imu_acc_z", 0.0)
        self.imu_pub.publish(imu)

        # Publish walking phase
        p = Phase()
        p.header.stamp = stamp
        zero_command = self.robot_status.get("zero_command", 0.0)
        walk_phase = self.robot_status.get("walk_phase", 0.0)
        if zero_command == 1:  # robot is standing
            p.phase = Phase.DOUBLE_STANCE
        else:
            if walk_phase < 0.02 or walk_phase > 0.98:
                p.phase = Phase.DOUBLE_STANCE
            elif 0.48 < walk_phase < 0.52:
                p.phase = Phase.DOUBLE_STANCE
            elif 0.02 <= walk_phase <= 0.48:
                p.phase = Phase.LEFT_STANCE
            elif 0.52 <= walk_phase <= 0.98:
                p.phase = Phase.RIGHT_STANCE
        self.phase_pub.publish(p)

        # Publish velocity estimator
        vel = Twist()
        vel.linear.x = self.robot_status.get("vel_x", 0.0)
        vel.linear.y = self.robot_status.get("vel_y", 0.0)
        vel.angular.z = self.robot_status.get("vel_z", 0.0)
        self.vel_estimator_pub.publish(vel)

        # Publish walk engine odometry
        self.walk_odom.header.stamp = stamp
        # self.walk_odom.pose.pose.position.x += self.robot_status.get("vel_x", 0.0) * 0.01
        self.walk_odom.pose.pose.position.x += self.robot_cmd["vel_x"] * 0.01
        # self.walk_odom.pose.pose.position.y += self.robot_status.get("vel_y", 0.0) * 0.01
        self.walk_odom.pose.pose.position.y += self.robot_cmd["vel_y"] * 0.01
        r, p, y = quat2euler(
            [
                self.walk_odom.pose.pose.orientation.w,
                self.walk_odom.pose.pose.orientation.x,
                self.walk_odom.pose.pose.orientation.y,
                self.walk_odom.pose.pose.orientation.z,
            ]
        )
        # y += self.robot_status.get("vel_z", 0.0) * 0.01
        y += self.robot_status.get("vel_yaw", 0.0) * 0.01

        q = euler2quat(r, p, y)
        self.walk_odom.pose.pose.orientation.w = q[0]
        self.walk_odom.pose.pose.orientation.x = q[1]
        self.walk_odom.pose.pose.orientation.y = q[2]
        self.walk_odom.pose.pose.orientation.z = q[3]
        self.walk_odom_pub.publish(self.walk_odom)

    def set_torque_cb(self, request: SetBool.Request, response: SetBool.Response):
        # Stub implementation — fill in with actual hardware interface
        self.get_logger().info("Not yet implements!!! Torque %s" % ("enabled" if request.data else "disabled"))
        response.success = True
        return response

    def cmd_vel_cb(self, twist: Twist):
        self.robot_cmd["vel_x"] = twist.linear.x
        self.robot_cmd["vel_y"] = twist.linear.y
        self.robot_cmd["vel_yaw"] = twist.angular.z
        # moving = twist.linear.x != 0.0 or twist.linear.y != 0.0 or twist.angular.z != 0.0
        # self.robot_cmd["loco_mode"] = "loco_walk" #if moving else "loco_walk_and_stand"


def main(args=None):
    rclpy.init(args=args)
    node = X02Control()
    rclpy.spin(node)
    node.ws.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
