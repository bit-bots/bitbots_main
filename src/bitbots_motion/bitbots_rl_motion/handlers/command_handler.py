import numpy as np
from bitbots_rl_motion.handlers.handler import Handler


class CommandHandler(Handler):
    def __init__(self, config):
        super().__init__(config)

        self._cmd_vel = None

    def get_command(self):
        command = np.array([self._cmd_vel.linear.x, self._cmd_vel.linear.y, self._cmd_vel.angular.z], dtype=np.float32)
        return command

    def cmd_vel_callback(self, msg):
        self._cmd_vel = msg
