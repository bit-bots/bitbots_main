import numpy as np
from soccer_vision_3d_msgs.msg import BallArray

from handlers.handler import Handler


class BallHandler(Handler):
    def __init__(self, node):
        self._node = node

        self._ball_pos = None

        self._ball_pos_sub = self._node.create_subscription(BallArray, "balls_relative", self._ball_pos_callback, 10)

    def _ball_pos_callback(self, msg):
        if msg.balls:
            self._ball_pos = msg.balls[0].center

    def has_data(self):
        return self._ball_pos is not None

    def get_ball_pos(self):
        ball_pos = np.array(
            [
                self._ball_pos.x,
                self._ball_pos.y,
            ],
            dtype=np.float32,
        )
        return ball_pos
