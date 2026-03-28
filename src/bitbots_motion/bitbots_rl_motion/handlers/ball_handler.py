from bitbots_rl_motion.handlers.handler import Handler


class BallHandler(Handler):
    def __init__(self, config):
        super().__init__(config)
        self.ball_pos = None

    def ball_pos_callback(self, msg):
        self.ball_pos = msg

    def get_ball_pos(self):
        return self.ball_pos
