from handlers.handler import Handler
import numpy as np

class BallHandler(Handler):
    def __init__(self, config):
        super().__init__(config)
        self._ball_pos = None

    def ball_pos_callback(self, msg):
        self._ball_pos = msg

    def has_data(self):
        return (self._ball_pos != None)

    def get_ball_pos(self):
        ball_pos = np.array(
            [
                self._ball_pos.pose.position.x,
                self._ball_pos.pose.position.y,
            ],
            dtype=np.float32,
        )
        return ball_pos
