import numpy as np


class HistoryBuffer:
    """Fixed-length history buffer for stacked observations.

    Mirrors the ``CircularBuffer`` used by the training onboard code: the buffer
    is lazily allocated on the first ``append`` and, while it is still empty, the
    first pushed value fills every slot (so a fresh history starts saturated with
    the first real observation instead of zeros). Subsequent pushes shift the
    buffer by one, keeping the newest value at index ``-1``. ``buffer`` therefore
    holds the frames oldest-first, which flattens to the order the policy expects.
    """

    def __init__(self, length: int):
        self._buffer: np.ndarray | None = None
        self._length = length
        self._num_pushes = 0

    def append(self, value: np.ndarray) -> None:
        value = np.asarray(value, dtype=np.float32)
        if self._buffer is None:
            self._buffer = np.zeros((self._length, *value.shape), dtype=np.float32)
        if self._num_pushes == 0:
            self._buffer[:] = value
        else:
            self._buffer = np.roll(self._buffer, -1, axis=0)
            self._buffer[-1] = value
        self._num_pushes += 1

    @property
    def buffer(self) -> np.ndarray:
        assert self._buffer is not None, "HistoryBuffer.append must be called before reading the buffer"
        return self._buffer

    def reset(self) -> None:
        if self._buffer is None:
            return
        self._buffer[:] = 0.0
        self._num_pushes = 0
