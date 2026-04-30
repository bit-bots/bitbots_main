import numpy as np


class ObservationHistory:
    """Rolling history buffer that mirrors HoST's observation stacking.

    HoST builds the actor input by stacking ``length`` consecutive one-step
    observations of dimension ``one_step_dim``. New frames are appended at the
    end and the oldest frame is dropped from the front. The buffer is
    initialised with zeros, matching the training-time buffer state right after
    a reset.
    """

    def __init__(self, length: int, one_step_dim: int):
        self._length = length
        self._one_step_dim = one_step_dim
        self._buffer = np.zeros(length * one_step_dim, dtype=np.float32)

    def reset(self) -> None:
        self._buffer.fill(0.0)

    def update(self, current_obs: np.ndarray) -> None:
        assert current_obs.shape == (self._one_step_dim,), (
            f"expected one-step obs of shape ({self._one_step_dim},), got {current_obs.shape}"
        )
        # shift left by one frame, append the new frame at the end
        self._buffer[: -self._one_step_dim] = self._buffer[self._one_step_dim :]
        self._buffer[-self._one_step_dim :] = current_obs.astype(np.float32, copy=False)

    def get(self) -> np.ndarray:
        return self._buffer

    @property
    def total_dim(self) -> int:
        return self._length * self._one_step_dim
