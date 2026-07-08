#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this


@launch_this
def audio(sample_rate: int = 48000):
    """
    Parameters
    ----------
    sample_rate : int
        The sample_rate with which the audio should be captured. Our code currently is not
        sample_rate agnostic, but this must be overwritable to start the audio_capturer_node
        on some audio drivers/devices if they do not support the default sample_rate of 16000.
    """
    bl = BetterLaunch()
    bl.node(
        "audio_common",
        "audio_capturer_node",
        "audio_capturer_node",
        params={"rate": sample_rate},
    )
