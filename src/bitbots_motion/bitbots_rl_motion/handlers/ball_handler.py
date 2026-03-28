from handlers.handler import Handler


class BallHandler(Handler):
    def __init__(self, config):
        super().__init__(config)
        self.ball_pos = None

    def ball_pos_callback(self, msg):
        self.ball_pos = msg

    def has_data(self):
        return (self.ball_pos != None)

    def get_ball_pos(self):
        return self.ball_pos
