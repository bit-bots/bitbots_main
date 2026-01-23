import time
from pathlib import Path
from threading import Lock

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Duration, Time
from ros2_numpy import numpify
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion

DEFAULT_LOG_FOLDER = Path("~/monitoring_logs").expanduser()

SPEED_THRESHOLD = 0.05  # m/s
BALL_DELTA_THRESHOLDS = ((SPEED_THRESHOLD * 0.95) ** 2, (SPEED_THRESHOLD / 0.98) ** 2)


class Monitoring(Node):
    def __init__(self, log_folder: Path = DEFAULT_LOG_FOLDER):
        # create node
        super().__init__("monitoring")
        use_sim_time_param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        self.set_parameters([use_sim_time_param])

        self.create_subscription(Bool, "kick", self.kick_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_cb, 10)
        self.create_subscription(ModelStates, "/model_states", qos_profile=10, callback=self.model_states_cb)

        log_folder.mkdir(exist_ok=True, parents=True)
        self.csv_file = self.open_file(log_folder)
        self.csv_file.write("index, time, event, data1, data2, data3\n")
        self.lock = Lock()
        self.index = 1

        self.last_goal_pose: PoseStamped = PoseStamped()
        self.last_model_states_time: Time = None
        self.last_robot_pose: Pose = Pose()
        self.last_ball_pose: Pose = Pose()
        self.ball_moving = False

    def open_file(self, log_folder: Path):
        base_name = time.strftime("%Y-%m-%dT%H:%M")
        i = 0
        while True:
            name = base_name if i == 0 else f"{base_name}-{i}"
            name += ".csv"
            if (log_folder / name).exists():
                i += 1
            else:
                break
        return (log_folder / name).open("w")

    def write_event(self, event: str, data1: str = "", data2: str = "", data3: str = "", time: Time | None = None):
        if time is None:
            time = self.get_clock().now()
        time = time.nanoseconds
        time = f"{time // 60_000_000_000:02}:{(time / 1_000_000_000) % 60:07.4f}"
        msg = ", ".join((time, event, data1, data2, data3))
        with self.lock:
            index = self.index
            self.index += 1
            self.csv_file.write(f"{index}, {msg}\n")
            self.csv_file.flush()

    def write_reduced_pose(self, event: str, pose: Pose, time: Time | None = None):
        yaw = euler_from_quaternion(numpify(pose.orientation))[2]
        self.write_event(event, f"{pose.position.x:.4f}", f"{pose.position.y:.4f}", f"{yaw:.3f}")

    def kick_cb(self, msg: Bool):
        goal_pose = self.last_goal_pose
        self.write_event("kick", ("left", "right")[msg.data])
        self.write_reduced_pose("kick_goal_pose", goal_pose.pose, goal_pose.header.stamp)
        self.write_reduced_pose("kick_robot_pos", self.last_robot_pose, self.last_model_states_time)
        self.write_reduced_pose("kick_ball_pos", self.last_ball_pose, self.last_model_states_time)

    def goal_pose_cb(self, msg: PoseStamped):
        self.last_goal_pose = msg

    def model_states_cb(self, msg: ModelStates):
        new_time = self.get_clock().now()
        time_delta = (
            new_time - self.last_model_states_time if self.last_model_states_time is not None else Duration(seconds=0.1)
        )
        if time_delta.nanoseconds < 1000:
            return
        self.last_model_states_time = new_time
        for i, name in enumerate(msg.name):
            if name == "amy":
                self.last_robot_pose = msg.pose[i]
            if name == "ball":
                self.ball_pose_cb(msg.pose[i], msg.twist[i])
                self.last_ball_pose = msg.pose[i]

    def ball_pose_cb(self, new_ball_pose: Pose, ball_twist: Twist):
        new = numpify(new_ball_pose.position)[:2]
        delta = numpify(ball_twist.linear)[:3]
        vel_sq = delta[:2].dot(delta[:2])
        if self.ball_moving:
            if vel_sq < BALL_DELTA_THRESHOLDS[0]:
                self.ball_moving = False
                self.write_event("ball_stopped", f"{new[0]:.4f}", f"{new[1]:.4f}")
                self.write_reduced_pose(
                    "ball_stopped_goal_pose", self.last_goal_pose.pose, self.last_goal_pose.header.stamp
                )
                self.write_reduced_pose("ball_stopped_robot_pose", self.last_robot_pose, self.last_model_states_time)
            else:
                self.write_event("ball_vel", f"{delta[0]:.4f}", f"{delta[1]:.4f}", f"{delta[2]:.4f}")
        else:
            if vel_sq > BALL_DELTA_THRESHOLDS[1]:
                self.ball_moving = True
                self.write_event("ball_started", f"{new[0]:.4f}", f"{new[1]:.4f}")
