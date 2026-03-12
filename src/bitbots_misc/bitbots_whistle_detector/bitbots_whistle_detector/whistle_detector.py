#! /usr/bin/env python3

import numpy as np
import rclpy
from audio_common_msgs.msg import AudioStamped
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty


class WhistleDetector(Node):
    def __init__(self) -> None:
        super().__init__("whistle_detector")
        self.logger = self.get_logger()

        self.whistle_publisher = self.create_publisher(Empty, "whistle_detected", 1)

        self.audio_buffer = np.array([], dtype=np.float32)
        self.sample_rate = 16000
        self.chunk_size = 512

        self.audio_sub = self.create_subscription(
            AudioStamped, "/audio", self.audio_cb, qos_profile=qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.02, self.process_audio)

        self.logger.info("Whistle detector initialized")

    def audio_cb(self, msg):
        audio_np = np.frombuffer(msg.audio.audio_data.int16_data, dtype=np.int16).astype(np.float32)
        audio_np /= 32768.0  # normalize, as from ints in range +/- 32768 to +/-1.0

        self.audio_buffer = np.concatenate([self.audio_buffer, audio_np])

        if len(self.audio_buffer) > self.chunk_size:
            self.audio_buffer = self.audio_buffer[-self.chunk_size :]

    def process_audio(self) -> None:
        if len(self.audio_buffer) < self.chunk_size:
            return

        audio = self.audio_buffer.copy()

        whistle_detected = self.detect_whistle(audio, self.sample_rate)

        if whistle_detected:
            msg = Empty()
            self.whistle_publisher.publish(msg)

    def detect_whistle(self, audio, sample_rate):
        spectrum = np.abs(np.fft.rfft(audio))
        freqs = np.fft.rfftfreq(len(audio), 1 / sample_rate)

        band = (freqs > 2000) & (freqs < 4500)  # Google: range of whistle frequencies

        whistle_energy = np.sum(spectrum[band])
        total_energy = np.sum(spectrum)

        if total_energy == 0:
            return False

        ratio = whistle_energy / total_energy

        return ratio > 0.6


def main(args=None) -> None:
    rclpy.init(args=args)

    node = WhistleDetector()
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
