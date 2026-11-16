"""Place the kickoff ball during SET and wait for the observed acknowledgement."""

from dataclasses import replace

from bitbots_auto_referee.rules.ball_placement import free_ball_position


class KickoffBall:
    def __init__(self, teleport_ball, field, event):
        self.teleport_ball = teleport_ball
        self.field = field
        self.event = event
        self.reset()

    def reset(self):
        self.armed = False
        self.placed = False
        self.future = None
        self.failed = False
        self._stop_owned = False
        self._blocked = False

    def arm(self):
        self.armed = True

    @property
    def waiting(self):
        return self.armed and not self.placed

    @property
    def owns_stop(self):
        return self._stop_owned

    def check(self, state, observation):
        if state.state in ("STATE_INITIAL", "STATE_FINISHED", "STATE_PLAYING"):
            self.reset()
            return state
        if state.state == "STATE_READY":
            self.arm()
            return state
        if state.state != "STATE_SET" or not self.waiting:
            return state
        if not state.stopped:
            self._stop_owned = True
        if not self.failed:
            try:
                if self.future is None:
                    position = free_ball_position((0.0, 0.0), observation, self.field)
                    if position is None:
                        if not self._blocked:
                            self.event("Anstoßballplatzierung blockiert; SET bleibt angehalten.")
                        self._blocked = True
                        return replace(state, stopped=True)
                    self._blocked = False
                    self.future = self.teleport_ball(*position, 0.0)
                if self.future.done():
                    result = self.future.result()
                    if not result.success:
                        raise RuntimeError(result.message)
                    if observation.step_number >= result.applied_step:
                        self.placed = True
                        self.event("Anstoßball in SET platziert und bestätigt.")
                        return replace(state, stopped=False) if self._stop_owned else state
            except Exception as error:
                self.failed = True
                self.event(f"Anstoßballplatzierung fehlgeschlagen; SET bleibt angehalten: {error}")
        return replace(state, stopped=True)
