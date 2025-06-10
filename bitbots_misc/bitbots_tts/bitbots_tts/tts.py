#!/usr/bin/env python3

import io
import traceback
import wave
from pathlib import Path

import numpy as np
import rclpy
import sounddevice as sd
from piper import PiperVoice
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher

from bitbots_msgs.msg import Audio

# Load the Piper voice
bb_tts_dir = Path(__file__).parent.parent / "model"  # TODO: check how to get nice relative paths
model_path = bb_tts_dir / "en_US-lessac-medium.onnx"
config_path = bb_tts_dir / "en_US-lessac-medium.onnx.json"
voice = PiperVoice.load(model_path, config_path=config_path, use_cuda=False)


def speak(text: str, publisher: Publisher, priority: int = 20, speaking_active: bool = True) -> None:
    """Utility method which can be used by other classes to easily publish a message."""
    if speaking_active:
        msg = Audio()
        msg.priority = priority
        msg.text = text
        publisher.publish(msg)


def say(text: str) -> None:
    """Use piper for speech synthesis and audio playback.
    This is also used for speaking the ip adress during startup."""
    synthesize_args = {
        "length_scale": 1.0,  # Phoneme length, if lower -> faster
        "noise_scale": 0.667,  # Generator noise, if lower -> more robotic
        "noise_w": 0.8,  # Phoneme width noise, if lower -> more robotic
        "sentence_silence": 0.1,  # seconds of silence after each sentence
    }
    with io.BytesIO() as buffer:
        with wave.open(buffer, "wb") as wav_file:
            voice.synthesize(text, wav_file, **synthesize_args)

        buffer.seek(0)
        with wave.open(buffer, "rb") as wav:
            framerate = wav.getframerate()
            sampwidth = wav.getsampwidth()
            nchannels = wav.getnchannels()
            nframes = wav.getnframes()
            audio_bytes = wav.readframes(nframes)

            # bytes to np array
            dtype_map = {1: np.int8, 2: np.int16, 4: np.int32}
            if sampwidth not in dtype_map:
                raise ValueError(f"Unsupported sample width: {sampwidth}")
            audio = np.frombuffer(audio_bytes, dtype=dtype_map[sampwidth])
            if nchannels > 1:
                audio = audio.reshape(-1, nchannels)

            sd.play(audio, samplerate=framerate, blocking=True)


class Speaker(Node):
    """
    Uses tts to say messages from the speak topic.
    """

    def __init__(self) -> None:
        """Initializes the node and the parameters."""
        super().__init__("tts_speaker")

        # Class Variables
        self.prio_queue: list[tuple[str, int]] = []
        self.speak_enabled = None
        self.print_say = None
        self.message_level = None

        # Initialize Parameters
        self.declare_parameter("print", True)
        self.declare_parameter("talk", True)
        self.declare_parameter("msg_level", 0)
        self.print_say: bool = self.get_parameter("print").value
        self.speak_enabled: bool = self.get_parameter("talk").value
        self.message_level: int = self.get_parameter("msg_level").value

        # Callback for parameter changes
        self.add_on_set_parameters_callback(self.on_set_parameters)

        # Subscribe to the speak topic
        self.create_subscription(Audio, "speak", self.speak_cb, 10, callback_group=MutuallyExclusiveCallbackGroup())

        # Start processing the queue
        self.create_timer(0.1, self.run_speaker, callback_group=MutuallyExclusiveCallbackGroup())

    def on_set_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        """Callback for parameter changes."""
        for parameter in parameters:
            if parameter.name == "print":
                self.print_say = parameter.value.bool_value
            elif parameter.name == "talk":
                self.speak_enabled = parameter.value.bool_value
            elif parameter.name == "msg_level":
                self.message_level = parameter.value.int_value
            else:
                self.get_logger().info("Unknown parameter: " + parameter.name)
        return SetParametersResult(successful=True)

    def run_speaker(self) -> None:
        """Continuously checks the queue and speaks the next message."""
        # Check if there is a message in the queue
        if len(self.prio_queue) > 0:
            # Get the next message and speak it
            text, _ = self.prio_queue.pop(0)
            try:
                say(text)
            except OSError:
                self.get_logger().error(str(traceback.format_exc()))

    def speak_cb(self, msg: Audio) -> None:
        """Handles incoming msg on speak topic."""
        # First decide if it's a file or a text
        text = msg.text
        if text is None:
            self.get_logger().warn("Speaker got message without content.")
        prio = msg.priority

        # If printing is enabled and it's a text, print it
        if self.print_say:
            self.get_logger().info("Said: " + text)

        # Check if the message is already in the queue or if it's priority is high enough to be added
        if self.speak_enabled and prio >= self.message_level and (text, prio) not in self.prio_queue:
            self.prio_queue.append((text, prio))
            self.prio_queue.sort(key=lambda x: x[1], reverse=True)


def main(args=None):
    rclpy.init(args=args)
    node = Speaker()
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
