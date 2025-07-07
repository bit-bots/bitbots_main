import time
from pathlib import Path
from threading import Lock

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Bool

DEFAULT_LOG_FOLDER = Path("~/monitoring_logs").expanduser()


class Monitoring(Node):
    def __init__(self, log_folder: Path = DEFAULT_LOG_FOLDER):
        # create node
        super().__init__("Monitoring")
        use_sim_time_param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        self.set_parameters([use_sim_time_param])
        self.create_subscription(Bool, "kick", self.kick_cb, 10)
        log_folder.mkdir(exist_ok=True, parents=True)
        self.csv_file = self.open_file(log_folder)
        self.csv_file.write("index, time, event, data1, data2, data3\n")
        self.lock = Lock()
        self.index = 1

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
        time = f"{time // 60_000_000_000}:{(time / 1_000_000_000) % 60:.2f}"
        msg = ", ".join((time, event, data1, data2, data3))
        with self.lock:
            index = self.index
            self.index += 1
            self.csv_file.write(f"{index}, {msg}\n")
            self.csv_file.flush()

    def kick_cb(self, msg: Bool):
        self.write_event("kick", ("left", "right")[msg.data])
