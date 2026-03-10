import numpy as np


class CommandHandler:
    def __init__(self):
        self._cmd_vel = None

    def get_data(self):
        command = np.array([self._cmd_vel.linear.x, self._cmd_vel.linear.y, self._cmd_vel.angular.z], dtype=np.float32)
        return command

    def cmd_vel_callback(self, msg):
        self._cmd_vel = msg
