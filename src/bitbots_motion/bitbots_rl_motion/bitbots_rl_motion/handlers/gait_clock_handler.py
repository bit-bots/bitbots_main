import numpy as np

from bitbots_rl_motion.handlers import Handler


class GaitClockHandler(Handler):
    """Gait clock of the Pi Plus AMP walk policy.

    During training the clock runs inside the environment. The policy has one
    extra action dimension (the last one) that sets the clock frequency:
    frequency = mid + half * tanh(action), with mid and half from the frequency
    range. After each policy step the phase increases by frequency * control_dt
    (modulo 1). The policy observes (sin, cos) of 2 * pi * phase.

    The observation of a step uses the phase before the update with the action
    of the same step. This is the same order as in the training.
    """

    def __init__(self, frequency_min: float, frequency_max: float, control_dt: float, initial_phase: float = 0.0):
        assert frequency_max >= frequency_min, "The maximum gait frequency must not be less than the minimum"
        assert control_dt > 0.0, "The control period must be positive"
        self._frequency_mid = 0.5 * (frequency_min + frequency_max)
        self._frequency_half = 0.5 * (frequency_max - frequency_min)
        self._control_dt = control_dt
        self._initial_phase = initial_phase % 1.0
        self._phase = self._initial_phase
        self._frequency = self._frequency_mid

    def has_data(self) -> bool:
        return True

    def reset(self) -> None:
        """Sets the phase to the initial phase and the frequency to the middle of the range."""
        self._phase = self._initial_phase
        self._frequency = self._frequency_mid

    def frequency_from_action(self, frequency_action: float) -> float:
        """Converts the frequency action of the policy to a frequency in Hz."""
        return float(self._frequency_mid + self._frequency_half * np.tanh(frequency_action))

    def advance(self, frequency_action: float) -> None:
        """Applies the frequency action of the current step and moves the phase forward."""
        self._frequency = self.frequency_from_action(frequency_action)
        self._phase = (self._phase + self._frequency * self._control_dt) % 1.0

    def get_phase(self) -> float:
        return self._phase

    def get_frequency(self) -> float:
        return self._frequency

    def get_obs(self) -> np.ndarray:
        """Returns (sin, cos) of the phase angle."""
        angle = 2.0 * np.pi * self._phase
        return np.array([np.sin(angle), np.cos(angle)], dtype=np.float32)
