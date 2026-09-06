#!/usr/bin/env python3
"""The stacked observations of the AMP policies depend on how the history is filled.

During training the history buffer backfills every slot with the first frame after a
reset, so the policy never sees zeros in place of frames that were not observed yet.
The runners rebuild the history on every activation, so they have to do the same.
"""

import numpy as np

from bitbots_rl_motion.history_buffer import HistoryBuffer


def test_the_first_frame_fills_the_whole_history():
    history = HistoryBuffer(4)

    history.append(np.array([1.0, 2.0]))

    assert history.buffer.shape == (4, 2)
    assert np.all(history.buffer == np.array([1.0, 2.0]))


def test_the_newest_frame_is_last():
    history = HistoryBuffer(4)

    history.append(np.array([1.0, 2.0]))
    history.append(np.array([3.0, 4.0]))

    # Oldest first, so flattening gives the frames in the order the policy expects
    assert np.allclose(history.buffer[-1], [3.0, 4.0])
    assert np.allclose(history.buffer[0], [1.0, 2.0])


def test_the_oldest_frame_is_dropped_once_the_history_is_full():
    history = HistoryBuffer(3)

    for step in range(5):
        history.append(np.array([float(step)]))

    assert np.allclose(history.buffer.flatten(), [2.0, 3.0, 4.0])


def test_a_reset_history_is_filled_by_the_next_frame_again():
    history = HistoryBuffer(4)
    history.append(np.array([1.0]))
    history.append(np.array([2.0]))

    history.reset()
    history.append(np.array([9.0]))

    # Not a single frame of the previous activation is left, and no zeros either
    assert np.all(history.buffer == 9.0)
