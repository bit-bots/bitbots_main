#!/usr/bin/env python3

import os
import traceback
from functools import partial

import numpy as np
import rclpy
import soundcard as sc
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher

from bitbots_msgs.msg import Audio
from bitbots_tts.supertonic.helper import load_text_to_speech, load_voice_style, timer


def speak(text: str, publisher: Publisher, priority: int = 20, speaking_active: bool = True) -> None:
    """Utility method which can be used by other classes to easily publish a message."""
    if speaking_active:
        msg = Audio()
        msg.priority = priority
        msg.text = text
        publisher.publish(msg)


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

        # Load the tts model
        conda_prefix = os.environ.get("CONDA_PREFIX", "")
        if not conda_prefix:
            raise ValueError(
                "CONDA_PREFIX environment variable not set! We now expect models to be shared as conda packages."
            )

        # Assemble model package name and look at its share directory
        model_path = os.path.join(conda_prefix, "share", "tts_supertonic")

        self.text_to_speech_engine = load_text_to_speech(os.path.join(model_path, "onnx"), use_gpu=True)

        # TODO make this configurable via parameters
        voice = "F2"
        steps = 5  # Number of diffusion steps, higher is better quality but also slower

        style = load_voice_style([os.path.join(model_path, "voice_styles", f"{voice}.json")])

        self.generate_speech = partial(self.text_to_speech_engine, style=style, total_step=steps)

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
                with timer("TTS Generation Time"):
                    wav_untrimmed, duration = self.generate_speech(f"{text}")
                wav = wav_untrimmed[0, : int(self.text_to_speech_engine.sample_rate * duration[0].item())]
                wav = np.concatenate([np.zeros(2000), wav])
                speaker = sc.default_speaker()
                with speaker.player(samplerate=self.text_to_speech_engine.sample_rate) as p:
                    p.play(wav)
                self.get_logger().info(
                    f"Finished speaking: {text} (Duration: {duration[0].item():.2f}s) Used device: {speaker.name}"
                )
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
